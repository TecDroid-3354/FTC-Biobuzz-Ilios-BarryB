package org.firstinspires.ftc.teamcode.opModes.calibration

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.sensors.ColorSensorV3.Companion.rgbToHsv
import org.firstinspires.ftc.teamcode.utils.sensors.Hsv
import org.firstinspires.ftc.teamcode.utils.sensors.ColorSensorConfig
import org.firstinspires.ftc.teamcode.utils.sensors.ColorSensorV3
import org.firstinspires.ftc.teamcode.utils.sensors.DetectedColor
import org.firstinspires.ftc.teamcode.utils.sensors.DetectedColor.NO_DETECTION
import org.firstinspires.ftc.teamcode.utils.sensors.DetectedColor.RED
import org.firstinspires.ftc.teamcode.utils.sensors.DetectedColor.BLUE
import org.firstinspires.ftc.teamcode.utils.sensors.DetectedColor.YELLOW
import org.firstinspires.ftc.teamcode.utils.sensors.HueRange
import java.util.Locale

private const val SENSOR_NAME = "indexerColor"
private const val SAMPLES = 10                 // reads averaged per capture

/** Live-editable from the Panels configurables tab. Doubles on purpose (safest type for Panels). */
@Configurable
object ColorCalibrationConfig {
    @JvmField var gain = 2.0
    @JvmField var ledEnabled = true
    @JvmField var minSaturation = 0.35
    @JvmField var minValue = 0.02
    @JvmField var hueTolerance = 20.0
}

private class Edge {
    private var prev = false
    fun rose(now: Boolean): Boolean = (now && !prev).also { prev = now }
}

private fun f(x: Number) = "%.3f".format(Locale.US, x.toDouble())

@TeleOp(name = "Color Sensor Calibration", group = "Calibration")
class ColorSensorCalibration: LinearOpMode() {

    private val config = ColorCalibrationConfig
    private lateinit var sensor: ColorSensorV3
    private val captured = mutableMapOf<DetectedColor, Hsv>() // NO_DETECTION = empty indexer
    private var page = 0
    private var message = "Press A with the indexer EMPTY to start."

    override fun runOpMode() {
        sensor = ColorSensorV3(
            hardwareMap,
            ColorSensorConfig(
                SENSOR_NAME
            )
        )
        val left = Edge(); val right = Edge(); val a = Edge(); val b = Edge()
        val x = Edge(); val y = Edge(); val rb = Edge()

        // Runs from INIT: do NOT press Play, just press Stop when done.
        while (!isStopRequested) {
            sensor.applyConfig(
                ColorSensorConfig(
                    name = SENSOR_NAME,
                    gain = config.gain.toFloat(),
                    ledEnabled = config.ledEnabled,
                    minSaturation = this@ColorSensorCalibration.config.minSaturation.toFloat(),
                    minValue = config.minValue.toFloat()
                )
            )
            // Re-apply the captured mean hues with the live tolerance, so tolerance edits act instantly.
            captured.forEach { (color, hsv) ->
                if (color != NO_DETECTION) {
                    sensor.setHueRange(color,
                        HueRange(
                            hsv.hue,
                            config.hueTolerance.toFloat()
                        )
                    )
                }
            }

            if (left.rose(gamepad1.dpad_left)) page = (page + 3) % 4
            if (right.rose(gamepad1.dpad_right)) page = (page + 1) % 4
            if (a.rose(gamepad1.a)) capture(NO_DETECTION)
            if (b.rose(gamepad1.b)) capture(RED)
            if (x.rose(gamepad1.x)) capture(BLUE)
            if (y.rose(gamepad1.y)) capture(YELLOW)
            if (rb.rose(gamepad1.right_bumper)) applySuggestions()

            telemetry.addData("Page (DPAD L/R)", "${page + 1}/4")
            telemetry.addData("Last action", message)
            when (page) {
                0 -> helpPage()
                1 -> livePage()
                2 -> capturesPage()
                else -> resultPage()
            }
            telemetry.update()
        }
    }

    /** Averages SAMPLES reads (less noise), then stores the resulting HSV. */
    private fun capture(color: DetectedColor) {
        var r = 0f; var g = 0f; var b = 0f
        repeat(SAMPLES) {
            val c = sensor.getRgb()
            r += c.red; g += c.green; b += c.blue
            sleep(20)
        }
        val hsv = rgbToHsv(r / SAMPLES, g / SAMPLES, b / SAMPLES)
        captured[color] = hsv
        val label = if (color == NO_DETECTION) "EMPTY" else color.name
        message = "Captured $label: hue ${f(hsv.hue)} sat ${f(hsv.saturation)} value ${f(hsv.value)}" +
            if (hsv.value > 0.95f) "  [!] CLIPPING, lower gain and recapture" else ""
    }

    /** Threshold = 1/3 of the way from the EMPTY reading to the dimmest ball: a ball is never missed. */
    private fun suggestion(selector: (Hsv) -> Float): Float? {
        val empty = selector(captured[NO_DETECTION] ?: return null)
        val dimmestBall = listOf(RED, BLUE, YELLOW)
            .mapNotNull { captured[it] }.minOfOrNull(selector) ?: return null
        return if (dimmestBall > empty) empty + (dimmestBall - empty) / 3f else null
    }

    private fun applySuggestions() {
        val v = suggestion { it.value }
        val s = suggestion { it.saturation }
        if (v != null) config.minValue = v.toDouble()
        if (s != null) config.minSaturation = s.toDouble()
        message = "minValue " + (v?.let { "set to ${f(it)}" } ?: "NOT set (needs EMPTY + a brighter ball)") +
            ", minSaturation " + (s?.let { "set to ${f(it)}" } ?: "NOT set (empty is as saturated as a ball, ignore it)")
    }

    private fun helpPage() {
        telemetry.addLine("=== HELP ===")
        telemetry.addLine("Edit gain, ledEnabled, minSaturation, minValue and hueTolerance live in Panels (Configurables).")
        telemetry.addLine("1) GAIN: put a pollen at its real spot in the indexer, go to page 2 and raise/lower gain until 'value' is 0.3-0.7. Near 1.0 = clipping = wrong hue.")
        telemetry.addLine("2) CAPTURE: indexer EMPTY -> A. Then each pollen: B = RED, X = BLUE, Y = YELLOW (Xbox/F310 layout). Each capture averages $SAMPLES reads.")
        telemetry.addLine("3) THRESHOLDS: RIGHT BUMPER sets minValue/minSaturation automatically (1/3 of the way from empty to the dimmest ball). Fine-tune in Panels if needed.")
        telemetry.addLine("4) TEST: on page 2 put each pollen (and nothing) in front; DETECTED must match. Widen/narrow hueTolerance in Panels, it applies live.")
        telemetry.addLine("5) Page 4 prints the config to paste into your code. If you change gain, recapture everything.")
    }

    private fun livePage() {
        val rgb = sensor.getRgb()
        val hsv = rgbToHsv(rgb.red, rgb.green, rgb.blue)
        val advice = when {
            hsv.value > 0.95f -> "CLIPPING: lower gain"
            hsv.value < 0.25f -> "dim: raise gain (ignore if indexer is empty)"
            else -> "good range (if a pollen is in front)"
        }
        telemetry.addLine("=== LIVE READING ===")
        telemetry.addData("RGB", "${f(rgb.red)}  ${f(rgb.green)}  ${f(rgb.blue)}")
        telemetry.addData("Hue (deg) / Sat / Value", "${f(hsv.hue)} / ${f(hsv.saturation)} / ${f(hsv.value)}")
        telemetry.addData("Gain advice", advice)
        telemetry.addData("DETECTED", sensor.getColor())
        telemetry.addLine("Hue 0-360 is the color; Sat is how colorful; Value is brightness (strongest channel).")
        telemetry.addLine("NO_DETECTION with a ball in front? Sat/Value are under their minimums, or hue is outside every range.")
    }

    private fun capturesPage() {
        telemetry.addLine("=== CAPTURED SAMPLES ===")
        listOf(
            NO_DETECTION to "EMPTY (A)", RED to "RED (B)",
            BLUE to "BLUE (X)", YELLOW to "YELLOW (Y)"
        ).forEach { (color, label) ->
            val h = captured[color]
            telemetry.addData(label, h?.let { "hue ${f(it.hue)}  sat ${f(it.saturation)}  value ${f(it.value)}" } ?: "not captured")
        }
        telemetry.addData("Suggested minValue", suggestion { it.value }?.let { f(it) } ?: "needs EMPTY + at least one ball")
        telemetry.addData("Suggested minSaturation", suggestion { it.saturation }?.let { f(it) } ?: "no separation (ok, rely on minValue)")
        telemetry.addData("Current minValue / minSat", "${f(config.minValue)} / ${f(config.minSaturation)}")
        telemetry.addLine("RIGHT BUMPER applies the suggestions. Hues are stored automatically as each color's mean.")
    }

    private fun resultPage() {
        telemetry.addLine("=== PASTE THIS INTO YOUR CODE ===")
        telemetry.addLine("ColorSensorConfig(name = \"$SENSOR_NAME\", gain = ${f(config.gain)}f, ledEnabled = ${config.ledEnabled},")
        telemetry.addLine("  minSaturation = ${f(config.minSaturation)}f, minValue = ${f(config.minValue)}f,")
        telemetry.addLine("  hueRanges = mapOf(")
        telemetry.addLine(sensor.calibrationSummary())
        telemetry.addLine("))")
        telemetry.addLine("Colors you did not capture still show placeholder defaults. Values are lost on restart, so paste them now.")
    }
}
