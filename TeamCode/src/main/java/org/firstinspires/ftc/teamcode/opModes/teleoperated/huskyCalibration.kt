package org.firstinspires.ftc.teamcode.opModes.teleoperated

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.vision.Husky
import kotlin.math.log


@TeleOp(name = "Husky Calibration", group = "TeleOp - Blue")


class HuskyCalibration : CommandOpMode() {
    lateinit var huskyCamera: Husky
    override fun initialize() {
        huskyCamera = Husky(hardwareMap)
    }

    override fun run() {
        super.run()

        huskyCamera.log(telemetry)
        telemetry.update()
    }


}
