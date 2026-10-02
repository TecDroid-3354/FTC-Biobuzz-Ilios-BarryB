package org.firstinspires.ftc.teamcode.systems

import org.firstinspires.ftc.robotcore.external.Supplier
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.ParallelCommandGroup
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.constants.DriveMultipliers
import org.firstinspires.ftc.teamcode.constants.FieldDimensions
import org.firstinspires.ftc.teamcode.constants.RobotTransformations
import org.firstinspires.ftc.teamcode.constants.ScoringTargets
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer
import org.firstinspires.ftc.teamcode.subsystems.intake.intakeDeploy.IntakeDeploy
import org.firstinspires.ftc.teamcode.subsystems.intake.intakeRollers.IntakeRollers
import org.firstinspires.ftc.teamcode.subsystems.mecanum.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheelHardStop.FlywheelHardStop
import org.firstinspires.ftc.teamcode.subsystems.shooter.hood.Hood
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.utils.extensions.toTranslation2d
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.teamcode.utils.units.LinearVelocity
import java.util.Optional
import java.util.function.BooleanSupplier
import kotlin.math.atan2
import kotlin.math.hypot

class Superstructure(
    private val hardwareMap: HardwareMap,
    private val controller: GamepadEx,
    private val alliance: Alliance
): SubsystemBase() {

    // ------------------------------------------ //
    // --------- Subsystem Declaration ---------- //
    // ------------------------------------------ //
    private lateinit var follower: Follower
    private lateinit var mecanum: Mecanum
    private lateinit var intakeDeploy: IntakeDeploy
    private lateinit var intakeRollers: IntakeRollers
    private lateinit var indexer: Indexer
    private lateinit var turret: Turret
    private lateinit var hood: Hood
    private lateinit var flywheel: Flywheel
    private lateinit var flywheelHardStop: FlywheelHardStop
    private lateinit var limelight: Limelight

    // ---------------------------------------------------------------- //
    // --------- Useful variables (Velocities, distances and dynamic targets) ---------- //
    // ---------------------------------------------------------------- //
    private val scoringTargets: ScoringTargets = ScoringTargets(alliance)
    private var robotToTurretPose: Pose2d = Pose2d()
    private var turretDistanceToTarget: Distance = Distance(0.0)
    private var robotRadialVelocityToTarget: LinearVelocity = LinearVelocity(0.0)
    private var robotTangentialVelocityToTarget: LinearVelocity = LinearVelocity(0.0)

    init {
        subsystemInitialization()
    }

    override fun periodic() {
        // Updates the robot to turret pose by retrieving the Mecanum's pose and transforming it by the Turret Transform2d
        robotToTurretPose = mecanum.getPose().transformBy(RobotTransformations.ROBOT_TO_TURRET_2D)

        // Updates the Turret's distance to the current scoring target.
        turretDistanceToTarget = Distance.fromMeters(
            hypot(
                getScoringTarget().get().x.minus(robotToTurretPose.x),
                getScoringTarget().get().y.minus(robotToTurretPose.y)
            )
        )

        // Updates the robot's radial velocity to the current scoring target
        robotRadialVelocityToTarget = mecanum.getRobotRadialVelocity(
            getScoringTarget().get()
        )

        // Updates the robot's tangential velocity to the current scoring target
        robotTangentialVelocityToTarget = mecanum.getRobotTangentialVelocity(
            getScoringTarget().get()
        )
    }

    private fun subsystemInitialization() {
        // Follower Initialization
        follower = Constants.createFollower(hardwareMap)
        // Mecanum Initialization
        mecanum = Mecanum(follower, controller, alliance)
        // Intake deploy Initialization
        intakeDeploy = IntakeDeploy(hardwareMap)
        // Intake Rollers Initialization
        intakeRollers = IntakeRollers(hardwareMap)
        // Indexer Initialization
        indexer = Indexer(hardwareMap)
        // Turret Initialization
        turret = Turret(hardwareMap)
        // Hood Initialization
        hood = Hood(hardwareMap)
        // Flywheel Initialization
        flywheel = Flywheel(hardwareMap)
        // Flywheel hard stop initialization
        flywheelHardStop = FlywheelHardStop(hardwareMap)
        // Limelight initialization
        limelight = Limelight(hardwareMap, mecanum) { mecanum.getRotation() }
    }

    // --------------- ----- -------- --------------- //
    // --------------- DRIVE COMMANDS --------------- //
    // --------------- ----- -------- --------------- //

    /**
     * Sets the [Mecanum] default command and its initial [pose] in teleoperated init.
     */
    fun setDriveDefaultCommandAndInitialPose(pose: Pose) {
        mecanum.defaultCommand = mecanum.driveFollowingDriverInput()
        mecanum.setPose(pose)
    }

    /**
     * Reduces the [Mecanum] by [DriveMultipliers.CONTROLLER_SOTM_LINEAR_MULTIPLIER] for x and y movement and
     * by [DriveMultipliers.CONTROLLER_SOTM_ANGULAR_MULTIPLIER] for angular movement.
     * Called for shooting on the move.
     */
    fun setDriveScaledSOTMCommand(): Command {
        return mecanum.driveFollowingScaledDriverInput(
            Optional.of(DriveMultipliers.CONTROLLER_SOTM_LINEAR_MULTIPLIER),
            Optional.of(DriveMultipliers.CONTROLLER_SOTM_ANGULAR_MULTIPLIER)
        )
    }

    /** Sets the [Mecanum] starting [pose] in autonomous */
    fun setStartingPose(pose: Pose) {
        mecanum.setPose(pose)
    }

    /**
     * Drives the [Mecanum] to a target [pose] in the field. Calls [Mecanum.followPathToTargetFieldPosition]
     * reach its target position.
     */
    fun setDriveFieldTargetPosition(pose: Pose2d): Command {
        return mecanum.followPathToTargetFieldPosition(pose, true, 1.0)
    }

    /** Checks whether the [Mecanum] is on the left side of the field. */
    fun isDriveOnLeftSide(): BooleanSupplier {
        return { mecanum.getPose().y >= FieldDimensions.LEFT_SIDE_END.inches }
    }

    /** Based on the robot's position on the field (left/right side) returns the current scoring target. */
    fun getScoringTarget(): Supplier<Translation2d> {
        return { if (isDriveOnLeftSide().asBoolean) scoringTargets.leftHivePose.toTranslation2d() else scoringTargets.rightHivePose.toTranslation2d() }
    }

    // --------------- -------- -------- --------------- //
    // --------------- SEQUENCE COMMANDS --------------- //
    // --------------- -------- -------- --------------- //

    /**
     * A [ParallelCommandGroup] which enables both the shooter and the hood, a [WaitUntilCommand] for
     * the flywheel and turret to reach its target. Once the condition is met, the pathway to the [Flywheel]
     * is cleared by the [FlywheelHardStop] and the [Indexer] is enabled. Allows SOTM.
     */
    fun setScoringSequence(): Command {
        return ParallelCommandGroup(
            setFlywheelCalculatedVelocity(),
            setHoodCalculatedAngle(),
            WaitUntilCommand { isFlywheelAtTarget() && isTurretAtTarget() }
                .andThen(clearPathToFlywheel())
                .andThen(WaitUntilCommand { isFlywheelHardStopAtTarget() })
                .andThen(setIndexerShootingPreset()),
        )
    }

    /**
     * A [ParallelCommandGroup] which stops the [Flywheel], [Indexer], moves the [Hood] to its home
     * position and the [FlywheelHardStop]blocks the pathway.
     */
    fun stopShootingSequence(): Command {
        return ParallelCommandGroup(
            stopFlywheel(),
            setHoodHomeAngle(),
            stopIndexer(),
            blockPathToFlywheel(),
        )
    }

    /**
     * A [ParallelCommandGroup] which sets the [IntakeDeploy] angle to intake from the floor, once it has
     * reached its target, the [IntakeRollers] will be enabled. The [Indexer] is enabled simultaneously
     * with a minimum velocity.
     */
    fun setFloorIntakeSequence(): Command {
        return ParallelCommandGroup(
            SequentialCommandGroup(
                setIntakeDeployFloorAngle(),
                WaitUntilCommand { isIntakeDeployAtTarget() },
                setIntakeRollersFloorVelocity()),
            setIndexerIdlePreset()
        )
    }

    /**
     * A [ParallelCommandGroup] which sets the [IntakeDeploy] angle to intake from the flower, once it has
     * reached its target, the [IntakeRollers] will be enabled with their target velocity.
     * The [Indexer] is enabled simultaneously with a minimum velocity.
     * The [Hood] is driven to its max position if scoring in the flower is desired.
     * The [Turret]'s target angle becomes zero, to score inside the flower.
     */
    fun setFlowerIntakeSequence(): Command {
        return ParallelCommandGroup(
            SequentialCommandGroup(
                setIntakeDeployFlowerAngle(),
                WaitUntilCommand { isIntakeDeployAtTarget() },
                setIntakeRollersFlowerVelocity()
            ),
            setIndexerIdlePreset(),
            setHoodPresetAngle(),
            setTurretZeroAngle()
        )
    }

    /**
     * A [ParallelCommandGroup] which deploys the [IntakeDeploy], stop the [IntakeRollers] and
     * sets the [Hood] to its home angle.
     */
    fun setIntakeIdle(): Command {
        return ParallelCommandGroup(
            setIntakeDeployFloorAngle(),
            stopIntakeRollers(),
            stopIndexer(),
            setHoodHomeAngle()
        )
    }

    /**
     * A [com.seattlesolvers.solverslib.command.RunCommand] which receives a [Mecanum] relative target angle, which is
     * calculated based on the current [ScoringTargets] and by compensating for the [robotTangentialVelocityToTarget]
     * based on the robot's virtual distance. It also receives the [Mecanum]'s rotation as an argument to convert the
     * chassis relative angle to a [Turret] angle.
     *
     */
    fun setTurretCalculatedAngleSequence(): Command {
        return setTurretCalculatedAngle(
            {
                mecanum.getAngleFromRobotToTarget(
                    if (isDriveOnLeftSide().asBoolean) scoringTargets.leftHivePose.toTranslation2d() else scoringTargets.rightHivePose.toTranslation2d(),
                ).minus(getTurretYawCorrection(
                    getVirtualDistance(turretDistanceToTarget)
                ))
            }, { mecanum.getRotation() },
        )
    }

    // --------------- ------- -------- --------------- //
    // --------------- INDEXER COMMANDS --------------- //
    // --------------- ------- -------- --------------- //


    /** Enables the configurable targets of the [Indexer]. You can edit them through the Panels UI */
    fun setIndexerConfigurableControl(): Command {
        return indexer.enableIndexerConfigurableVelocity()
    }

    /** Enables the preset velocity for shooting value of the [Indexer]. Value can be found in [org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets] */
    fun setIndexerShootingPreset(): Command {
        return indexer.enableIndexerShootingVelocity()
    }

    /** Enables the preset idle velocity value of the [Indexer]. Value can be found in [org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets] */
    fun setIndexerIdlePreset(): Command {
        return indexer.enableIndexerIdleVelocity()
    }

    /** Stops the [Indexer] */
    fun stopIndexer(): Command {
        return indexer.stopIndexer()
    }

    /** Returns whether the [Indexer] has reached its velocity target */
    fun isIndexerAtTarget(): Boolean {
        return indexer.getIsAtTarget()
    }

    // --------------- ------- -------- --------------- //
    // ------------ INTAKE DEPLOY COMMANDS ------------ //
    // --------------- ------- -------- --------------- //


    /** Enables the configurable targets of the [IntakeDeploy]. You can edit them through the Panels UI */
    fun setIntakeDeployConfigurableControl(): Command {
        return intakeDeploy.setIntakeDeployConfigurableAngle()
    }

    /** Enables the preset floor intake angle value of the [IntakeDeploy]. Value can be found in [org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets] */
    fun setIntakeDeployFloorAngle(): Command {
        return intakeDeploy.setIntakeDeployFloorAngle()
    }

    /** Enables the preset flower intake angle value of the [IntakeDeploy]. Value can be found in [org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets] */
    fun setIntakeDeployFlowerAngle(): Command {
        return intakeDeploy.setIntakeDeployFlowerAngle()
    }

    /** Returns whether the [IntakeDeploy] has reached its angle target */
    fun isIntakeDeployAtTarget(): Boolean {
        return intakeDeploy.getIsAtTarget()
    }

    // --------------- ------- -------- --------------- //
    // ------------ INTAKE ROLLERS COMMANDS ----------- //
    // --------------- ------- -------- --------------- //

    /** Enables the configurable targets of the [IntakeRollers]. You can edit them through the Panels UI */
    fun setIntakeRollersConfigurableControl(): Command {
        return intakeRollers.enableIntakeRollersConfigurableVelocity()
    }

    /** Enables the preset floor intake velocity value of the [IntakeRollers]. Value can be found in [org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets] */
    fun setIntakeRollersFloorVelocity(): Command {
        return intakeRollers.enableIntakeRollersFloorVelocity()
    }

    /** Enables the preset flower intake velocity value of the [IntakeRollers]. Value can be found in [org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets] */
    fun setIntakeRollersFlowerVelocity(): Command {
        return intakeRollers.enableIntakeRollersFlowerVelocity()
    }

    /** Stops the [IntakeRollers] */
    fun stopIntakeRollers(): Command {
        return intakeRollers.stopIntakeRollers()
    }

    /** Returns whether the [IntakeRollers] has reached its velocity target */
    fun isIntakeRollersAtTarget(): Boolean {
        return intakeRollers.getIsAtTarget()
    }

    // --------------- ----- -------- --------------- //
    // ------------- FLYWHEEL COMMANDS -------------- //
    // --------------- ----- -------- --------------- //

    /** Uses the TOF interpolation charts and the robot's [robotRadialVelocityToTarget] to calculate
     * the virtual distance */
    private fun getVirtualDistance(flywheelDistanceToTarget: Distance): Distance {
        var virtualDistance: Distance = flywheelDistanceToTarget

        val currentRadialVelocity: LinearVelocity = robotRadialVelocityToTarget

        for (cycle in 1 .. 3) {
            val tof = flywheel.getCalculatedScoringTimeOfFlight(virtualDistance)
            val virtualDistanceMeters = flywheelDistanceToTarget.meters.minus(currentRadialVelocity.mps.times(tof.seconds))
            virtualDistance = Distance.fromMeters(virtualDistanceMeters)
        }

        return virtualDistance
    }

    /** Enables the configurable targets of the [Flywheel]. You can edit them through the Panels UI */
    fun setFlywheelConfigurableControl(): Command {
        return flywheel.setFlywheelConfigurableVelocity()
    }

    /** Enables the [Flywheel] with a calculated velocity based on the virtual distance and the robot's radial velocity */
    fun setFlywheelCalculatedVelocity(): Command {
        return flywheel.setFlywheelCalculatedScoringVelocity { getVirtualDistance(turretDistanceToTarget) }
    }

    /** Stops the [Flywheel] */
    fun stopFlywheel(): Command {
        return flywheel.stopFlywheel()
    }

    /** Returns whether the [Flywheel] has reached its velocity target */
    fun isFlywheelAtTarget(): Boolean {
        return flywheel.getIsAtTarget()
    }

    // --------------- ----- -------- --------------- //
    // --------- FLYWHEEL HARD STOP COMMANDS -------- //
    // --------------- ----- -------- --------------- //

    /** Enables the configurable targets of the [FlywheelHardStop]. You can edit them through the Panels UI */
    fun setFlywheelHardStopConfigurableControl(): Command {
        return flywheelHardStop.setFlywheelHardStopConfigurableAngle()
    }

    /** Clears the pathway to the [Flywheel] */
    fun clearPathToFlywheel(): Command {
        return flywheelHardStop.setFlywheelHardStopClearingAngle()
    }

    /** Blocks the pathway to the [Flywheel] */
    fun blockPathToFlywheel(): Command {
        return flywheelHardStop.setFlywheelHardStopBlockingAngle()
    }

    /** Returns whether the [FlywheelHardStop] has reached its angle target */
    fun isFlywheelHardStopAtTarget(): Boolean {
        return flywheelHardStop.getIsAtTarget()
    }

    // --------------- ----- -------- --------------- //
    // ---------------- HOOD COMMANDS --------------- //
    // --------------- ----- -------- --------------- //


    /** Enables the configurable targets of the [Hood]. You can edit them through the Panels UI */
    fun setHoodConfigurableControl(): Command {
        return hood.setHoodConfigurableAngle()
    }

    /** Enables the [Hood] with a calculated angle based on the virtual distance and the robot's radial velocity */
    fun setHoodCalculatedAngle(): Command {
        return hood.setHoodCalculatedAngle { turretDistanceToTarget }
    }

    /** Sets the [Hood] to its home angle (0.0°) */
    fun setHoodHomeAngle(): Command {
        return hood.setHoodHomeAngle()
    }

    /** Sets the [Hood] to its preset angle (90.0°) */
    fun setHoodPresetAngle(): Command {
        return hood.setHoodPresetAngle()
    }

    /** Returns whether the [Hood] has reached its angle target */
    fun isHoodAtTarget(): Boolean {
        return hood.getIsAtTarget()
    }

    // --------------- ----- -------- --------------- //
    // -------------- TURRET COMMANDS --------------- //
    // --------------- ----- -------- --------------- //

    /** Calculates the angle the pollen will acquire once ejected from the turret due to the
     * effect of [robotTangentialVelocityToTarget], the offset is ought to be applied as
     * (theta - deltaTheta), where theta is the angle from the turret to the target
     * @param flywheelVirtualDistanceToTarget the turret's distance to the target */
    private fun getTurretYawCorrection(flywheelVirtualDistanceToTarget: Distance): Rotation2d {
        val projectileVelocityMps = flywheelVirtualDistanceToTarget.meters
            .div(flywheel.getCalculatedScoringTimeOfFlight(flywheelVirtualDistanceToTarget).seconds)
        val tangentialVelocity: LinearVelocity = robotTangentialVelocityToTarget
        val correction = atan2(tangentialVelocity.mps, projectileVelocityMps)

        return Rotation2d(correction)
    }

    /** Sets the turret default command, which targets the hive depending on the field zone the robot's in. */
    fun setTurretDefaultCommand() {
        turret.defaultCommand = setTurretCalculatedAngleSequence()
    }

    /** Enables the configurable targets of the [Turret]. You can edit them through the Panels UI */
    fun setTurretConfigurableAngle(): Command {
        return turret.setTurretConfigurableAngle()
    }

    /** Calculates the [Turret] relative angle to a target by subtracting the chassis current rotation by the
     * chassis target angle to the target */
    fun setTurretCalculatedAngle(chassisRelativeTargetAngle: Supplier<Rotation2d>, chassisCurrentRotation: Supplier<Rotation2d>): Command {
        return turret.setCalculatedTurretAngle(chassisRelativeTargetAngle, chassisCurrentRotation)
    }

    /** Sets the turret to its zero angle */
    fun setTurretZeroAngle(): Command {
        return turret.setTurretZeroAngle()
    }

    /** Returns whether the [Turret] has reached its angle target */
    fun isTurretAtTarget(): Boolean {
        return turret.getIsAtTarget()
    }

    // --------------- ----- -------- --------------- //
    // ---------------- END RUNNABLE --------------- //
    // --------------- ----- -------- --------------- //

    /** Returns a [Runnable] which assigns [PoseStorage.autonomousEndPose] the [Follower]'s last value */
    fun onEnd(): Runnable {
        return {
            PoseStorage.autonomousEndPose = follower.pose()
        }
    }
}