package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utils.localization.TagObservation
import org.firstinspires.ftc.teamcode.utils.localization.VisionFrame
import org.firstinspires.ftc.teamcode.utils.units.nowSeconds

/**
 * The Limelight is used as an AprilTag DETECTOR. It reports where each tag is relative to the camera;
 * turning that into a robot position (turret angle, pivot offset, moving tags, latency) happens in
 * [org.firstinspires.ftc.teamcode.utils.localization.PoseEstimator], where we know the turret angle at the
 * exact moment the picture was taken. Nothing depends on the Limelight's web-UI robot pose or field map.
 */
@Suppress("JoinDeclarationAndAssignment")
class Limelight(
    hardwareMap: HardwareMap,
    private val visionConsumer: VisionConsumer,
    private val isFilteringDisabled: Boolean = true
): SubsystemBase() {

    private var limelight: Limelight3A
    private var lastResultTimestamp = -1.0

    init {
        limelight = hardwareMap.get(Limelight3A::class.java, LimelightConstants.Identification.LIMELIGHT_ID)
        limelight.setPollRateHz(LimelightConstants.Configuration.POLL_RATE_HZ)
        limelight.pipelineSwitch(LimelightConstants.Configuration.APRIL_TAG_PIPELINE)
    }
    override fun periodic() {
        if (isFilteringDisabled) return

        val result = limelight.latestResult ?: return
        if (!result.isValid) return

        val timestamp = result.timestamp
        if (timestamp == lastResultTimestamp) return
        lastResultTimestamp = timestamp

        if (result.staleness > LimelightConstants.Configuration.STALENESS_THRESHOLD) return

        val fiducials = result.fiducialResults
        if (fiducials.isEmpty()) return

        val tags = ArrayList<TagObservation>(fiducials.size)
        for (i in fiducials.indices) {
            val fiducial = fiducials[i]
            val p = fiducial.targetPoseCameraSpace.position.toUnit(DistanceUnit.INCH)
            tags.add(TagObservation(fiducial.fiducialId, right = p.x, down = p.y, out = p.z))
        }

        val ageMs = result.staleness + result.captureLatency + result.targetingLatency +
                LimelightConstants.Configuration.LATENCY_CALIBRATION_MS
        visionConsumer.accept(VisionFrame(nowSeconds() - (ageMs / 1000.0), tags))
    }

    fun start(): Runnable {
        return { limelight.start() }
    }

    fun pause(): Runnable {
        return { limelight.pause() }
    }

    fun interface VisionConsumer {
        fun accept(frame: VisionFrame)
    }
}
