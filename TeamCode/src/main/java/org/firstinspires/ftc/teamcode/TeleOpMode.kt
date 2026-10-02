package org.firstinspires.ftc.teamcode

import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.utils.Alliance

open class TeleOpMode(val alliance: Alliance): CommandOpMode() {
    lateinit var controller: GamepadEx
    lateinit var robot: Robot

    override fun initialize() {
        super.reset()
        controller = GamepadEx(gamepad1)
        robot = Robot(alliance, hardwareMap, controller, telemetry)
        robot.initTeleOp()
    }

    override fun initialize_loop() {
        robot.initLoop()
    }

    override fun preRun() {
        robot.preTeleOp()
    }

    override fun run() {
        robot.run()
        robot.printTelemetry(robot.pTelemetry)
    }

    override fun end() {
        robot.onEnd()
    }
}