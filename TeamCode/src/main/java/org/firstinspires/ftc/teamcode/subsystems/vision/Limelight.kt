package org.firstinspires.ftc.teamcode.subsystems.vision

import com.bylazar.telemetry.TelemetryManager
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
    private val visionConsumer: VisionConsumer
): SubsystemBase() {

    private var limelight: Limelight3A
    private var lastResultTimestamp = -1.0

    init {
        limelight = hardwareMap.get(Limelight3A::class.java, LimelightConstants.Identification.LIMELIGHT_ID)
        limelight.setPollRateHz(LimelightConstants.Configuration.POLL_RATE_HZ)
        limelight.pipelineSwitch(LimelightConstants.Configuration.APRIL_TAG_PIPELINE)
    }

    override fun periodic() {
        val result = limelight.latestResult ?: return
        if (!result.isValid) return

        // The Limelight is polled faster than it produces frames: never feed the same frame twice.
        val timestamp = result.timestamp
        if (timestamp == lastResultTimestamp) return
        lastResultTimestamp = timestamp

        if (result.staleness > LimelightConstants.Configuration.STALENESS_THRESHOLD) return

        val tags = result.fiducialResults.map { fiducial ->
            // Tag center in CAMERA space. Axis convention is verified by LimelightAxisCalibration.
            val p = fiducial.targetPoseCameraSpace.position.toUnit(DistanceUnit.INCH)
            TagObservation(fiducial.fiducialId, right = p.x, down = p.y, out = p.z)
        }
        if (tags.isEmpty()) return

        // When was the picture taken, on the robot's clock?
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
