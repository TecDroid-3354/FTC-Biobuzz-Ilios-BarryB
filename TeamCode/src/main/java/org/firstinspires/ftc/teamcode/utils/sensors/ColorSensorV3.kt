package org.firstinspires.ftc.teamcode.utils.sensors

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.SwitchableLight
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

enum class DetectedColor { RED, BLUE, YELLOW, NO_DETECTION }

/** Normalized channels, each roughly 0..1 (depends on gain and distance). */
data class Rgb(val red: Float, val green: Float, val blue: Float)

/** hue in degrees 0..360, saturation 0..1, value 0..1 (value = brightest channel, same scale as Rgb). */
data class Hsv(val hue: Float, val saturation: Float, val value: Float)

/** A color is matched when the hue is within [hue - hueTolerance, hue + hueTolerance] (wraps around 360). */
data class HueRange(val hue: Float, val hueTolerance: Float)

// Placeholder values: they MUST be calibrated with your real pollen under your real lighting.
// NO_DETECTION intentionally has no range.
val DEFAULT_HUE_RANGES: Map<DetectedColor, HueRange> = mapOf(
    DetectedColor.RED to HueRange(hue = 0f, hueTolerance = 20f),
    DetectedColor.YELLOW to HueRange(hue = 70f, hueTolerance = 20f),
    DetectedColor.BLUE to HueRange(hue = 220f, hueTolerance = 25f)
)

data class ColorSensorConfig(
    val name: String,                 // device name in the Robot Configuration
    val gain: Float = 2f,             // sensor gain, tune for your mounting distance
    val ledEnabled: Boolean = true,   // sensor's white LED, keeps readings consistent
    val minSaturation: Float = 0.35f, // below this the reading is treated as "no color" (grey/empty)
    val minValue: Float = 0.02f,      // below this the reading is too dark to trust (empty indexer)
    val hueRanges: Map<DetectedColor, HueRange> = DEFAULT_HUE_RANGES
)

class ColorSensorV3(hardwareMap: HardwareMap, initialConfig: ColorSensorConfig) {

    var config: ColorSensorConfig = initialConfig
        private set

    private var sensor: NormalizedColorSensor

    // Copy so calibration can change ranges without touching the shared config/defaults.
    private val hueRanges: MutableMap<DetectedColor, HueRange> = initialConfig.hueRanges.toMutableMap()

    init {
        sensor = hardwareMap.get(NormalizedColorSensor::class.java, initialConfig.name)

        sensor.gain = initialConfig.gain
        if (sensor is SwitchableLight) {
            (sensor as SwitchableLight).enableLight(initialConfig.ledEnabled)
        }
    }

    /**
     * Updates gain, LED and thresholds at runtime (used for live tuning).
     * Hardware is only written when gain/LED actually changed. Ignores newConfig.name and
     * newConfig.hueRanges (use setHueRange / calibrate for those).
     */
    fun applyConfig(newConfig: ColorSensorConfig) {
        if (newConfig.gain != config.gain) sensor.gain = newConfig.gain
        if (newConfig.ledEnabled != config.ledEnabled) {
            (sensor as? SwitchableLight)?.enableLight(newConfig.ledEnabled)
        }
        config = newConfig
    }

    /** Normalized RGB (one I2C read). */
    fun getRgb(): Rgb {
        val c = sensor.normalizedColors
        return Rgb(c.red, c.green, c.blue)
    }

    /** HSV computed from the normalized RGB (one I2C read). */
    fun getHsv(): Hsv {
        val rgb = getRgb()
        return rgbToHsv(rgb.red, rgb.green, rgb.blue)
    }

    /**
     * Returns the matching color, or NO_DETECTION if nothing matched
     * (empty indexer, too dark, too grey, or hue outside every range).
     */
    fun getColor(): DetectedColor {
        val hsv = getHsv() // single read per call
        if (hsv.saturation < config.minSaturation || hsv.value < config.minValue) {
            return DetectedColor.NO_DETECTION
        }

        return hueRanges.entries
            .map { (color, range) -> color to hueDistance(hsv.hue, range.hue) - range.hueTolerance }
            .filter { it.second <= 0f }                 // inside the range
            .minByOrNull { it.second }?.first           // if ranges overlap, pick the most "inside" one
            ?: DetectedColor.NO_DETECTION
    }

    /**
     * Point the sensor at a pollen of [color] and call this to set the mean hue
     * to the current reading. Tolerance is kept unless you pass a new one.
     * Lives in memory only: it resets when the OpMode restarts, so copy the result
     * of [calibrationSummary] into your config.
     */
    fun calibrate(color: DetectedColor, hueTolerance: Float? = null) {
        val tolerance = hueTolerance ?: hueRanges[color]?.hueTolerance ?: 20f
        setHueRange(color, HueRange(getHsv().hue, tolerance))
    }

    fun setHueRange(color: DetectedColor, range: HueRange) {
        require(color != DetectedColor.NO_DETECTION) { "NO_DETECTION has no hue range" }
        hueRanges[color] = range
    }

    /** Copy-pasteable lines for DEFAULT_HUE_RANGES (or your own config map). */
    fun calibrationSummary(): String =
        hueRanges.entries.joinToString("\n") { (color, range) ->
            "DetectedColor.$color to HueRange(hue = ${range.hue}f, hueTolerance = ${range.hueTolerance}f),"
        }

    fun getHueRange(color: DetectedColor): HueRange? = hueRanges[color]

    companion object {
        fun rgbToHsv(r: Float, g: Float, b: Float): Hsv {
            val maxC = max(r, max(g, b))
            val minC = min(r, min(g, b))
            val delta = maxC - minC

            val rawHue = when {
                delta == 0f -> 0f
                maxC == r -> 60f * (((g - b) / delta) % 6f)
                maxC == g -> 60f * (((b - r) / delta) + 2f)
                else -> 60f * (((r - g) / delta) + 4f)
            }
            val hue = if (rawHue < 0f) rawHue + 360f else rawHue
            val saturation = if (maxC == 0f) 0f else delta / maxC
            return Hsv(hue, saturation, maxC)
        }
    }

    /** Shortest angular distance between two hues (handles the 359° -> 0° wrap, important for RED). */
    private fun hueDistance(a: Float, b: Float): Float {
        val d = abs(a - b) % 360f
        return min(d, 360f - d)
    }
}
