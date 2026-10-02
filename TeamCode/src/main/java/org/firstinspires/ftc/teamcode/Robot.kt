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
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.TecDroidRobot
import org.firstinspires.ftc.teamcode.utils.autonomous.PoseStorage

class Robot(
    private val alliance: Alliance,
    private val hardwareMap: HardwareMap,
    private val controller: GamepadEx,
    private val telemetry: Telemetry
): TecDroidRobot(hardwareMap) {

    /* Declare your Pedro Pathing's Follower here */
    private lateinit var follower: Follower
    /* Declare your subsystems here */
    private lateinit var drive: Mecanum

    init {
        subsystemInitialization()
    }

    /* Initialize your subsystems and follower here */
    override fun subsystemInitialization() {
        // Follower initialization
        follower = Constants.createFollower(hardwareMap)
        // Subsystem initialization
        drive = Mecanum(follower, controller, alliance)
    }

    /* Runs indefinitely after the init button on the DS is pressed. Stops when play button is pressed */
    override fun initLoop() {}

    /* Initialize your teleop controller commands here */
    override fun initTeleOp() {
        // Chassis default command
        drive.setPose(PoseStorage.autonomousEndPose)
        drive.defaultCommand = drive.driveFollowingDriverInput()

        // Configure Control Bindings
        configureBindings()
    }

    /**
     * Runs once before the main loop of the robot [loopTeleOp]
     */
    override fun preTeleOp() {}

    /**
     * Runs periodically, useful for updating variables or configurables.
     */
    override fun loopTeleOp() {}

    /**
     * Runs once when init is pressed during auto.
     * Initialize auto commands and set STARTING pose.
     */
    override fun initAuto(startingPose: Pose) {
        drive.setPose(startingPose)
    }

    /**
     * Runs once on the end of OpModes.
     */
    override fun onEnd() {
        PoseStorage.autonomousEndPose = follower.pose()
    }

    /**
     * Runs inside the main loop of the robot. Print telemetry ONLY.
     */
    override fun printTelemetry(telemetryManager: TelemetryManager) {
        // Add here your telemetry. telemetryManager.addData("Name", variable)
        telemetryManager.update(telemetry)
    }

    /* Common method to follow any path */
    override fun followPathCMD(path: Path, holdEnd: Boolean, maxPower: Double): Command {
        return drive.followPathCMD(path, holdEnd, maxPower)
    }

    /**
     * Write here your controller.button() commands.
     */
    override fun configureBindings() {}
}