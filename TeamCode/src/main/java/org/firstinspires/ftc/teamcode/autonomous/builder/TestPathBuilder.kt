package org.firstinspires.ftc.teamcode.autonomous.builder

import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.autonomous.paths.testPaths.TestPath1
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.autonomous.PoseStorage

/**
 * The following class was created with the objective of serving as a template for an autonomous routine.
 * This class is similar to the [org.firstinspires.ftc.teamcode.TeleOpMode], however, the [GamepadEx] and its command are inexistent here.
 * Declared below the [org.firstinspires.ftc.teamcode.Robot] is the [autonomousCommand] which is a [SequentialCommandGroup] and is the only command
 * that will be executed along the whole routine. You can nest inside the [autonomousCommand] several [org.firstinspires.ftc.teamcode.Robot.followPathCMD],
 * [SequentialCommandGroup], [ParallelDeadlineGroup], [InstantCommand], etc.
 * You'll need to create an instance of a org.firstinspires.ftc.teamcode.autonomous.paths.examplePaths.Paths class like [ExamplePaths] and initialize it with an [Alliance]. Once initialized
 * the [TestPathBuilder] can be initialized with both [Alliance.RED] or [Alliance.BLUE] and paths will be mirrored flawlessly.
 * Complete each [TODO] inside [TestPathBuilder] and the class will be ready to become the inheritor of an [com.qualcomm.robotcore.eventloop.opmode.Autonomous].
 */
open class TestPathBuilder(private val alliance: Alliance): CommandOpMode() {

    // The LogiTech controller
    private lateinit var controller: GamepadEx
    // Robot's declaration
    private lateinit var robot: Robot
    private lateinit var paths: TestPath1
    // Autonomous command declaration
    private lateinit var autonomousCommand: SequentialCommandGroup

    // Executed after the init button is pressed.
    override fun initialize() {
        super.reset()
        controller = GamepadEx(gamepad1)
        robot = Robot(alliance, hardwareMap, controller, telemetry)

        paths = TestPath1(alliance)

        robot.initAuto(paths.start)

        // Chain commands inside the autonomous command //

        autonomousCommand = SequentialCommandGroup(
            robot.followPathCMD(paths.path1(), false, 0.8),
            robot.followPathCMD(paths.path2(), false, 1.0),
//            robot.followPathCMD(paths.path3(), true, 1.0),
//            robot.followPathCMD(paths.path4(), false, 0.8),
//            robot.followPathCMD(paths.path5(), false, 1.0),
//            robot.followPathCMD(paths.path6(), true, 1.0),
//            robot.followPathCMD(paths.path7(), false, 1.0)
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