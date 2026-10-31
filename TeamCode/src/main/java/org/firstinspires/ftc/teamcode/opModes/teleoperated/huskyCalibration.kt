package org.firstinspires.ftc.teamcode.opModes.teleoperated

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.vision.Husky

@TeleOp(name = "Husky Calibration")
class HuskyCalibration : LinearOpMode() {
    override fun runOpMode() {
        val huskyCamera = Husky(hardwareMap)



        while (!isStopRequested){
            huskyCamera.log(telemetry)

            telemetry.update()
        }
    }


}