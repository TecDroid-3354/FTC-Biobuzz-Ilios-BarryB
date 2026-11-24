package org.firstinspires.ftc.teamcode.autonomous.autos

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.autonomous.paths.SoloPath
import org.firstinspires.ftc.teamcode.systems.Superstructure
import org.firstinspires.ftc.teamcode.utils.Alliance

open class SoloAuto(private val alliance: Alliance): CommandOpMode() {

    private lateinit var batterySensor: VoltageSensor
    private lateinit var controller: GamepadEx
    // Robot's declaration
    private lateinit var robot: Robot
    private lateinit var paths: SoloPath

    // Autonomous command declaration
    private lateinit var autonomousCommand: SequentialCommandGroup

    // Executed after the init button is pressed.
    override fun initialize() {
        super.reset()
        controller = GamepadEx(gamepad1)
        robot = Robot(alliance, hardwareMap, controller, telemetry)

        batterySensor =  hardwareMap.voltageSensor.iterator().next()

        paths = SoloPath(alliance)

        robot.initAuto(paths.start)

        // Chain commands inside the autonomous command //

        autonomousCommand = SequentialCommandGroup(
            robot.superstructure.setScoringSequence(),
            WaitCommand(2000),
            robot.superstructure.stopShootingSequence(),
            robot.superstructure.setFloorIntakeSequence(),
            robot.followPathCMD(paths.path1(), false, 0.8),
            robot.superstructure.setIntakeIdle(),
            robot.followPathCMD(paths.path2(), false, 1.0),
            robot.followPathCMD(paths.path3(), false, 1.0),
            robot.followPathCMD(paths.path4(), false, 0.8),
            robot.followPathCMD(paths.path5(), false, 1.0),
            robot.followPathCMD(paths.path6(), false, 1.0),
            robot.followPathCMD(paths.path7(), false, 1.0),
            robot.followPathCMD(paths.path7(), false, 1.0),
            robot.followPathCMD(paths.path8(), true, 1.0),
            robot.followPathCMD(paths.path9(), true, 1.0),
            robot.followPathCMD(paths.path10(), true, 1.0),

        )
    }

    // Executed indefinitely after the init button is pressed
    override fun initialize_loop() {
        robot.initLoop()
    }

    override fun preRun() {
        robot.preAuto(autonomousCommand)
    }

    // Executed after the play button is pressed
    override fun run() {
        robot.run()
        robot.printTelemetry(robot.pTelemetry)
    }

    // Executed when the OpMode ends
    override fun end() {
        robot.onEnd()
    }
}