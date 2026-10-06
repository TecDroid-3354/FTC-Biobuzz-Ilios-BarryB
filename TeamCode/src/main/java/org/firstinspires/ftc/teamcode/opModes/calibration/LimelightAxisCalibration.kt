package org.firstinspires.ftc.teamcode.opModes.calibration

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightConstants

/**
 * Prints each tag's position in the Limelight's CAMERA space (inches).
 * The estimator assumes: x = right, y = down, z = out of the lens.
 * Put a tag straight ahead of the camera and check: z ~ distance, x ~ 0.
 * Slide the tag to the camera's RIGHT: x must grow. Raise the tag: y must get more NEGATIVE.
 * If any of that is flipped, fix the mapping in Limelight.periodic (one line), nothing else.
 */
@TeleOp(group = "Calibration", name = "Limelight Axis Calibration")
class LimelightAxisCalibration : LinearOpMode() {
    override fun runOpMode() {
        val limelight = hardwareMap.get(Limelight3A::class.java, LimelightConstants.Identification.LIMELIGHT_ID)
        limelight.pipelineSwitch(LimelightConstants.Configuration.APRIL_TAG_PIPELINE)
        limelight.start()
        waitForStart()
        while (opModeIsActive()) {
            val result = limelight.latestResult
            if (result == null || !result.isValid) {
                telemetry.addLine("no valid result")
            } else {
                telemetry.addData("staleness ms", result.staleness)
                result.fiducialResults.forEach { f ->
                    val p = f.targetPoseCameraSpace.position.toUnit(DistanceUnit.INCH)
                    telemetry.addData("tag ${f.fiducialId}", "right=%.1f  down=%.1f  out=%.1f", p.x, p.y, p.z)
                }
            }
            telemetry.update()
        }
        limelight.stop()
    }
}