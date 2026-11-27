package org.firstinspires.ftc.teamcode.autonomous.autos

import com.pedropathing.math.Pose
import com.qualcomm.hardware.ams.AMSColorSensor
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.ParallelCommandGroup
import com.seattlesolvers.solverslib.command.ParallelRaceGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.autonomous.paths.TwoTipParkPaths
import org.firstinspires.ftc.teamcode.constants.FieldDimensions
import org.firstinspires.ftc.teamcode.utils.Alliance

open class TwoTipParkBuilder(private val alliance: Alliance): CommandOpMode() {

    // The LogiTech controller
    private lateinit var controller: GamepadEx
    // Robot's declaration
    private lateinit var robot: Robot

    private lateinit var paths: TwoTipParkPaths
    // Autonomous command declaration
    private lateinit var autonomousCommand: SequentialCommandGroup

    // Executed after the init button is pressed.
    override fun initialize() {
        super.reset()
        controller = GamepadEx(gamepad1)
        robot = Robot(alliance, hardwareMap, controller, telemetry)

        paths = TwoTipParkPaths(alliance)

        robot.initAuto(paths.start)

        // Chain commands inside the autonomous command //
        autonomousCommand = SequentialCommandGroup(
            robot.superstructure.setAutoScoringSequence(),
            ParallelDeadlineGroup(
                SequentialCommandGroup(
                    robot.followPathCMD(paths.path1(), true, 0.35),
                    robot.followPathCMD(paths.path2(), true, 0.8),
                    robot.followPathCMD(paths.path3(), true, 0.8)
                ),
                robot.superstructure.toggleAndIntakeSequence()
            ),
            robot.superstructure.setIntakeIdle(),
            ParallelDeadlineGroup(
                robot.followPathCMD(paths.path4(), true , 1.0),
                SequentialCommandGroup(
                    WaitUntilCommand(robot.superstructure.isDriveReadyToEnableFlywheel()),
                    robot.superstructure.setFlywheelIdleAutoVelocity()
                )
            ),
            robot.superstructure.setConfigurableScoringSequence().andThen(WaitCommand(1400)).andThen(robot.superstructure.stopShootingSequence()),
            robot.followPathCMD(paths.path5(), true, 1.0),
            ParallelDeadlineGroup(
                WaitCommand(3000),
                SequentialCommandGroup(
                    robot.followPathCMD(paths.path6(), true, 0.8),
                    WaitCommand(1000),
                    robot.followPathCMD(paths.path7(), true, 1.0),
                    robot.followPathCMD(paths.path8(), true, 0.6)
                ),
                robot.superstructure.setIntakeSequence()
            ),
            robot.superstructure.stopIntakeRollers(),
            robot.superstructure.setAutoScoringSequence(),
            robot.followPathCMD(paths.path9(), true, 1.0)
        )
    }

    // Executed indefinitely after the init button is pressed
    override fun initialize_loop() {
        robot.initLoop()
    }

    override fun preRun() {
        // Schedule autonomous command
        autonomousCommand.schedule()
    }

    // Executed after the play button is pressed
    override fun run() {
        robot.run()
        robot.onEnd()
        robot.printTelemetry(robot.pTelemetry)
    }

    // Executed when the OpMode ends
    override fun end() {
        robot.onEnd()
    }
}