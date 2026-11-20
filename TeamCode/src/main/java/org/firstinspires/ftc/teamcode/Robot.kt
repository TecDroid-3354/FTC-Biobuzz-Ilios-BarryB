package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.systems.Superstructure
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.TecDroidRobot
import org.firstinspires.ftc.teamcode.utils.extensions.circle
import org.firstinspires.ftc.teamcode.utils.extensions.leftBumper
import org.firstinspires.ftc.teamcode.utils.extensions.leftTrigger
import org.firstinspires.ftc.teamcode.utils.extensions.onFalse
import org.firstinspires.ftc.teamcode.utils.extensions.onTrue
import org.firstinspires.ftc.teamcode.utils.extensions.rightBumper
import org.firstinspires.ftc.teamcode.utils.extensions.rightTrigger
import org.firstinspires.ftc.teamcode.utils.extensions.start
import org.firstinspires.ftc.teamcode.utils.extensions.triangle
import org.firstinspires.ftc.teamcode.utils.extensions.whileTrue
import org.firstinspires.ftc.teamcode.utils.units.Voltage

class Robot(
    alliance: Alliance,
    hardwareMap: HardwareMap,
    private val controller: GamepadEx,
    private val telemetry: Telemetry
): TecDroidRobot(hardwareMap) {

    /* Superstructure Declaration */
    private var superstructure: Superstructure

    private val batterySensor: VoltageSensor = hardwareMap.voltageSensor.iterator().next()

    init {
        superstructure = Superstructure(hardwareMap, controller, alliance, pTelemetry, batterySensor)
    }

    override fun preAuto(command: Command) {
        superstructure.preAuto(command).run()
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
        superstructure.preTeleop().run()
    }

    /* Initialize your teleop controller commands here */
    override fun initTeleOp(autoEndPose: Pose) {
        // Chassis default command
        // TODO Check why PoseStorage doesn't work
        superstructure.setPose(Pose(8.5, 8.2, 0.0))
        superstructure.setDriveDefaultCommand()

        superstructure.setTurretDefaultCommand()

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
        superstructure.logFlywheel()
        superstructure.logOdometry()
        telemetryManager.update()
    }

    /* Common method to follow any path */
    override fun followPathCMD(path: Path, holdEnd: Boolean, maxPower: Double): Command {
        return superstructure.followPathCommand(path, holdEnd, maxPower)
    }

    /**
     * Write here your controller.button() commands.
     */
    override fun configureBindings() {

//        controller.circle()
//            .onTrue(superstructure.setDropDownFlowerAngle())
//            .onFalse(superstructure.setDropDownFloorAngle())
//
//        controller.dpadRight()
//            .whileTrue(superstructure.driveToRightPark())
//
//        controller.dpadLeft()
//            .whileTrue(superstructure.driveToLeftPark())

        controller.start()
            .onTrue(InstantCommand({ superstructure.setPose(Pose(8.5, 8.2, 0.0)) }))

        controller.leftTrigger()
            .onTrue(superstructure.setConfigurableScoringSequence())//.alongWith(superstructure.setControllerRumbleRepeated()))
            .onFalse(superstructure.stopShootingSequence())


        controller.rightTrigger()
            .onTrue(superstructure.setScoringSequence())//.alongWith(superstructure.setControllerRumbleRepeated()))
            .onFalse(superstructure.stopShootingSequence())//.alongWith(superstructure.stopControllerRumble()))

        controller.rightBumper()
            .whileTrue(superstructure.setFloorIntakeSequence())
            .onFalse(superstructure.setIntakeIdle())
    }
}