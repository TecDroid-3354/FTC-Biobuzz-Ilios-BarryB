package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.mecanum.Mecanum
import org.firstinspires.ftc.teamcode.systems.Superstructure
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.TecDroidRobot
import org.firstinspires.ftc.teamcode.utils.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.utils.extensions.circle
import org.firstinspires.ftc.teamcode.utils.extensions.dpadLeft
import org.firstinspires.ftc.teamcode.utils.extensions.dpadRight
import org.firstinspires.ftc.teamcode.utils.extensions.leftBumper
import org.firstinspires.ftc.teamcode.utils.extensions.leftTrigger
import org.firstinspires.ftc.teamcode.utils.extensions.whileTrue

@Suppress("JoinDeclarationAndAssignment")
class Robot(
    alliance: Alliance,
    hardwareMap: HardwareMap,
    private val controller: GamepadEx,
    private val telemetry: Telemetry
): TecDroidRobot(hardwareMap) {

    /* Superstructure Declaration */
    private var superstructure: Superstructure

    init {
        superstructure = Superstructure(hardwareMap, controller, alliance, pTelemetry)
    }

    override fun preAuto(command: Command) {
        superstructure.preAuto(command)
    }

    /**
     * Runs once when init is pressed during auto.
     * Initialize auto commands and set STARTING pose.
     */
    override fun initAuto(startingPose: Pose) {
        superstructure.setPose(startingPose)
    }

    /* Runs indefinitely after the init button on the DS is pressed. Stops when play button is pressed */
    override fun initLoop() {}

    /**
     * Runs once before the main loop of the robot [loop]
     */
    override fun preTeleOp() {
        // TODO Uncomment when limelight is connected
        //superstructure.preTeleop().run()
    }

    /* Initialize your teleop controller commands here */
    override fun initTeleOp(autoEndPose: Pose) {
        // Chassis default command
        // TODO Check why PoseStorage doesn't work
        superstructure.setPose(Pose(9.8, 103.7206, Math.PI / 2))
        superstructure.setDriveDefaultCommand()

        // Configure Control Bindings
        configureBindings()
    }

    /**
     * Runs periodically, useful for updating variables or configurables.
     */
    override fun loop() {}

    /**
     * Runs once on the end of OpModes.
     */
    override fun onEnd() {
        superstructure.onEnd().run()
    }

    /**
     * Runs inside the main loop of the robot. Print telemetry ONLY.
     */
    override fun printTelemetry(telemetryManager: TelemetryManager) {
        // Add here your telemetry. telemetryManager.addData("Name", variable)
        superstructure.logOdometry()
        telemetryManager.update(telemetry)
    }

    /* Common method to follow any path */
    override fun followPathCMD(path: Path, holdEnd: Boolean, maxPower: Double): Command {
        return superstructure.followPathCommand(path, holdEnd, maxPower)
    }

    /**
     * Write here your controller.button() commands.
     */
    override fun configureBindings() {
        controller.leftBumper()
            .whileTrue(superstructure.driveToNearestFlower())

        controller.dpadRight()
            .whileTrue(superstructure.driveToRightPark())

        controller.dpadLeft()
            .whileTrue(superstructure.driveToLeftPark())
    }
}