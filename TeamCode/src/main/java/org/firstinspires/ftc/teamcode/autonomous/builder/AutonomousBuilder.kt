package org.firstinspires.ftc.teamcode.autonomous.builder

import com.pedropathing.math.Pose
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.utils.Alliance

/**
 * The following class was created with the objective of serving as a template for an autonomous routine.
 * This class is similar to the [org.firstinspires.ftc.teamcode.TeleOpMode], however, the [GamepadEx] and its command are inexistent here.
 * Declared below the [org.firstinspires.ftc.teamcode.Robot] is the [autonomousCommand] which is a [SequentialCommandGroup] and is the only command
 * that will be executed along the whole routine. You can nest inside the [autonomousCommand] several [org.firstinspires.ftc.teamcode.Robot.followPathCMD],
 * [SequentialCommandGroup], [ParallelDeadlineGroup], [InstantCommand], etc.
 * You'll need to create an instance of a org.firstinspires.ftc.teamcode.autonomous.paths.examplePaths.Paths class like [ExamplePaths] and initialize it with an [Alliance]. Once initialized
 * the [AutonomousBuilder] can be initialized with both [Alliance.RED] or [Alliance.BLUE] and paths will be mirrored flawlessly.
 * Complete each [TODO] inside [AutonomousBuilder] and the class will be ready to become the inheritor of an [com.qualcomm.robotcore.eventloop.opmode.Autonomous].
 */
open class AutonomousBuilder(private val alliance: Alliance): CommandOpMode() {

    // The LogiTech controller
    private lateinit var controller: GamepadEx
    // Robot's declaration
    private lateinit var robot: Robot
    // TODO Declare an instance of your org.firstinspires.ftc.teamcode.autonomous.paths.examplePaths.Paths class

    // Autonomous command declaration
    private lateinit var autonomousCommand: SequentialCommandGroup

    // Executed after the init button is pressed.
    override fun initialize() {
        super.reset()
        controller = GamepadEx(gamepad1)
        robot = Robot(alliance, hardwareMap, controller, telemetry)
        // TODO Initialize the instance of your org.firstinspires.ftc.teamcode.autonomous.paths.examplePaths.Paths class

        // TODO Set starting pose based on the autonomous to follow. Replace the empty Pose.
        robot.initAuto(Pose(0.0, 0.0))

        // Chain commands inside the autonomous command //
        // TODO IMPORTANT. Fill out the requirements of each command nested in here, if not, the robot won't follow any path.
        autonomousCommand = SequentialCommandGroup(
            // TODO Write here your commands
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
        robot.printTelemetry(robot.pTelemetry)
    }

    // Executed when the OpMode ends
    override fun end() {
        robot.onEnd()
    }
}