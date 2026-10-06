package org.firstinspires.ftc.teamcode.opModes.calibration

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.constants.LocalizationConstants
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightConstants
import org.firstinspires.ftc.teamcode.utils.localization.PoseEstimator
import org.firstinspires.ftc.teamcode.utils.localization.TagObservation

/**
 * Bench check for the whole chain (tag layout, axes, tag size, mount numbers, frame).
 *
 * 1. Put the robot at a spot you measured by hand and point it with the turret locked at a known angle.
 * 2. Type that pose into KNOWN_* below (Pedro frame: inches, origin at the field corner, heading counter-clockwise from +x).
 * 3. Run. For every visible tag you get the position it implies under each hive state.
 *    PASS: the right state is within ~2 in and its height residual is ~0 (the other state is ~15 in off in height).
 *    Errors of tens of inches: wrong tag size in the Limelight pipeline (needs 3.25 in = 82.55 mm),
 *    wrong frame (x/y swapped or mirrored), wrong tag ID order, or wrong mount numbers.
 */
@TeleOp(group = "Calibration", name = "Tag Map Check")
class TagMapCheck : LinearOpMode() {
    private val KNOWN_X = 40.0
    private val KNOWN_Y = 40.0
    private val KNOWN_HEADING_DEG = 45.0
    private val KNOWN_TURRET_DEG = 0.0

    override fun runOpMode() {
        val limelight = hardwareMap.get(Limelight3A::class.java, LimelightConstants.Identification.LIMELIGHT_ID)
        limelight.pipelineSwitch(LimelightConstants.Configuration.APRIL_TAG_PIPELINE)
        limelight.start()
        val estimator = PoseEstimator(
            LocalizationConstants.mount,
            LocalizationConstants.tagMap,
            LocalizationConstants.estimatorConfig
        )
        waitForStart()
        while (opModeIsActive()) {
            val result = limelight.latestResult
            if (result == null || !result.isValid || result.fiducialResults.isEmpty()) {
                telemetry.addLine("no tags in view")
            } else {
                val tags = result.fiducialResults.map {
                    val p = it.targetPoseCameraSpace.position.toUnit(DistanceUnit.INCH)
                    TagObservation(it.fiducialId, p.x, p.y, p.z)
                }
                estimator.diagnose(tags, KNOWN_X, KNOWN_Y, Math.toRadians(KNOWN_HEADING_DEG), Math.toRadians(KNOWN_TURRET_DEG))
                    .forEach {
                        telemetry.addData(
                            "tag ${it.id} state ${if (it.candidateIndex == 0) "+30" else "-30"}",
                            "x=%.1f y=%.1f  off by %.1f in  height residual %.1f in",
                            it.robotX, it.robotY, it.errorInches, it.heightResidualInches
                        )
                    }
            }
            telemetry.update()
        }
        limelight.stop()
    }
}