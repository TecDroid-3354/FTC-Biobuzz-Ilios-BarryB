package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.pedropathing.controllers.PIDController
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.follower.Follower
import com.pedropathing.follower.ManualDrive
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.seattlesolvers.solverslib.geometry.Vector2d
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand
import com.skeletonarmy.marrow.weaver.PathConfig
import com.skeletonarmy.marrow.weaver.Weaver
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.constants.DriveMultipliers
import org.firstinspires.ftc.teamcode.constants.RobotDimensions
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.Zones
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight
import org.firstinspires.ftc.teamcode.systems.TurretCameraMount
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.extensions.toPedroPath
import org.firstinspires.ftc.teamcode.utils.extensions.toPoint
import org.firstinspires.ftc.teamcode.utils.extensions.toPose2d
import org.firstinspires.ftc.teamcode.utils.localization.AprilTagMap
import org.firstinspires.ftc.teamcode.utils.units.LinearVelocity
import org.firstinspires.ftc.teamcode.utils.localization.PoseEstimator
import org.firstinspires.ftc.teamcode.utils.localization.VisionFrame
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.nowSeconds
import java.util.Optional
import kotlin.math.abs
import kotlin.math.atan2

class Mecanum(
    private val follower: Follower,
    private val controller: GamepadEx,
    private val alliance: Alliance,
    private val poseEstimator: PoseEstimator,
    private val turretAngle: () -> Angle
): SubsystemBase(), Limelight.VisionConsumer {

    constructor(follower: Follower,
                controller: GamepadEx,
                alliance: Alliance) : this(follower, controller, alliance,
                PoseEstimator(TurretCameraMount(), AprilTagMap()), { Angle(0.0) }) {
                    this.isFilterDisabled = true
                }

    private var isFilterDisabled = false
    private var angleController: PIDController = PIDController(SubsystemControlGains.MECANUM_ANGLE_PID.p, SubsystemControlGains.MECANUM_ANGLE_PID.i, SubsystemControlGains.MECANUM_ANGLE_PID.d)
    private var lastDriveAngle: Angle = Angle(0.0)

    init {
        // Initialization code
        configureWeaver()
    }

    /**
     * Runs in every loop. Follower and telemetry get updated
     */
    override fun periodic() {
        follower.update()

        if (isFilterDisabled.not()) {
            val odometry = follower.pose()
            poseEstimator.updateOdometry(nowSeconds(), odometry.x(), odometry.y(), odometry.heading(), turretAngle().radians)
        }

        // Update angle controller
        if (angleController.kP != SubsystemControlGains.MECANUM_ANGLE_PID.p
            || angleController.kI != SubsystemControlGains.MECANUM_ANGLE_PID.i
            || angleController.kD != SubsystemControlGains.MECANUM_ANGLE_PID.p) {
            angleController = PIDController(SubsystemControlGains.MECANUM_ANGLE_PID.p, SubsystemControlGains.MECANUM_ANGLE_PID.i, SubsystemControlGains.MECANUM_ANGLE_PID.d)
        }
    }

    /**
     * Retrieves the [controller]'s axis readings and converts them into robot's velocity.
     * The axis get multiplied by each [MecanumConstants.Control] Multiplier and its respective alliance multiplier.
     * @return a [RunCommand] which set the [Follower]'s TeleOp drive to the [controller]'s axis.
     */
    fun driveFollowingScaledDriverInput(driveScalar: Optional<Double>, headingScalar: Optional<Double>): Command {
        return RunCommand({
            val fieldCentricDrive = ManualDrive.fieldCentric(
                -controller.leftY * DriveMultipliers.FORWARD_VELOCITY_MULTIPLIER * alliance.multiplier * driveScalar.orElse(1.0),
                controller.leftX * DriveMultipliers.LATERAL_VELOCITY_MULTIPLIER * alliance.multiplier * driveScalar.orElse(1.0),
                controller.rightX * DriveMultipliers.TURN_VELOCITY_MULTIPLIER * headingScalar.orElse(1.0),
                follower.pose().heading()
            )

            follower.manual(fieldCentricDrive)
        })
            .addRequirements(this)
    }

    fun driveFollowingDriverInput(): Command {
        return driveFollowingScaledDriverInput(Optional.empty(), Optional.empty())
    }

    fun driveFieldRelativeScaled(driveScalar: Optional<Double>): Command {
        return RunCommand({
            // Calculate angle from joystick
            val angleFromJoystick = getAngleFromJoystick(controller.rightX, controller.rightY)
            // Store drive powers, turn is left at zero as ManualDrive.headingLock will calculate it.
            val forward = -controller.leftY * DriveMultipliers.FORWARD_VELOCITY_MULTIPLIER * alliance.multiplier * driveScalar.orElse(1.0)
            val strafe = controller.leftX * DriveMultipliers.LATERAL_VELOCITY_MULTIPLIER * alliance.multiplier * driveScalar.orElse(1.0)

            val drivePowers = DrivePowers(forward, strafe, 0.0)

            // Calculate the final powers to lock heading using the angleController and the calculated angle.
            // The follower only gives velocity, it is not commanded within ManualDrive.headingLock()
            val headingLockPower = ManualDrive.headingLock(follower, angleController, drivePowers, angleFromJoystick.radians)

            // Transform it to field relative
            val fieldRelativePowers = ManualDrive.fieldCentric(forward, strafe, headingLockPower.turn(), follower.pose().heading())
            // Drive manually using the headingLockPowers
            follower.manual(fieldRelativePowers)
        })
            .addRequirements(this)
    }

    fun driveFieldRelative(): Command {
        return driveFieldRelativeScaled(Optional.empty())
    }

    private fun getAngleFromJoystick(x: Double, y: Double): Angle {
        if (abs(-y) < 0.3 && abs(x) < 0.3) return lastDriveAngle;

        val atan2Rad = atan2(
            -y * alliance.multiplier, x * alliance.multiplier
        ) + Math.PI / 2

        lastDriveAngle = Angle.fromRadians(atan2Rad)

        return lastDriveAngle
    }

    /**
     * Gets the Follower's current position.
     * @return a [Pose2D] containing the robot's current position in the standard FTC Coordinates
     */
    fun getPose(): Pose {
        val odometry = follower.pose()
        if (!poseEstimator.isInitialized || isFilterDisabled) return odometry
        return Pose(poseEstimator.x, poseEstimator.y, odometry.heading())
    }

    /**
     * Gets the Follower's current position.
     * @return a [Pose2D] containing the robot's current position in the standard FTC Coordinates
     */
    fun getRawPose(): Pose {
        return follower.pose()
    }

    /**
     * Gets the [Follower]'s rotation component.
     * @return a [Rotation2d] as the robot's current heading in radians.
     */
    fun getRotation(): Rotation2d {
        return Rotation2d(getPose().heading())
    }

    /**
     * Gets the current [Follower]'s velocity as a [ChassisSpeeds].
     * This represents the velocity of the robot in the field.
     * @return the current in the field's frame
     */
    fun getFieldRelativeVelocity(): ChassisSpeeds {
        return ChassisSpeeds(follower.velocity().vx, follower.velocity().vy, follower.velocity().omega)
    }

    /**
     * Gets the current [Follower]'s velocity as a [ChassisSpeeds].
     * This represents the velocity of the robot.
     * @return the current robot's velocity in the robot's frame
     */
    fun getRobotRelativeVelocity(): ChassisSpeeds {
        return ChassisSpeeds(follower.twist().vx, follower.twist().vy, follower.twist().omega)
    }

    /**
     * Constructs a vector from the robot to a target and returns the projection of the velocity vector onto the distance unit vector.
     * If the result is positive, then the robot is driving towards the target.
     * If the result is negative, then the robot is driving away from the target.
     */
    fun getRobotRadialVelocity(fieldToTarget: Translation2d): LinearVelocity {
        val fieldRelativeVelocity = getFieldRelativeVelocity()
        val robotToTargetVector = fieldToTarget.minus(getPose().toPose2d().translation)
        val robotToTargetDistance = robotToTargetVector.norm

        if (robotToTargetDistance < 1e-5) return LinearVelocity(0.0)

        val radialUnitTranslation = robotToTargetVector.div(robotToTargetDistance)
        val radialUnitVector = Vector2d(radialUnitTranslation.x, radialUnitTranslation.y)

        val velocityVector = Vector2d(fieldRelativeVelocity.vxMetersPerSecond, fieldRelativeVelocity.vyMetersPerSecond)
        val radialVectorMagnitude = velocityVector.dot(radialUnitVector)

        return LinearVelocity.fromMps(radialVectorMagnitude)
    }

    /**
     * Constructs a vector from the robot to a target, the rotates it by 90.0 degrees and returns the projection of the velocity vector onto the tangential unit vector.
     * If the result is positive, then the robot is driving towards the target.
     * If the result is negative, then the robot is driving away from the target.
     */
    fun getRobotTangentialVelocity(fieldToTarget: Translation2d): LinearVelocity {
        val fieldRelativeSpeeds = getFieldRelativeVelocity()

        val robotToTargetVector = fieldToTarget.minus(getPose().toPose2d().translation)
        val robotToTargetDistance = robotToTargetVector.norm

        if (robotToTargetDistance < 1e-5) return LinearVelocity(0.0)

        val radialUnitTranslation = robotToTargetVector.div(robotToTargetDistance)
        val tangentialUnitTranslation = radialUnitTranslation.rotateBy(Rotation2d.fromDegrees(90.0))
        val tangentialUnitVector = Vector2d(tangentialUnitTranslation.x, tangentialUnitTranslation.y)

        val velocityVector = Vector2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
        val tangentialVectorMagnitude = velocityVector.dot(tangentialUnitVector)

        return LinearVelocity.fromMps(tangentialVectorMagnitude)
    }

    /**
     * Constructs a vector from the robot to a target and returns its angle plus an [Optional] [Rotation2d]
     * @return the angle of the vector plus the offset
     */
    fun getAngleFromRobotToTarget(fieldToTarget: Translation2d): Rotation2d {
        val robotToTargetVector = fieldToTarget.minus(getPose().toPose2d().translation)

        val targetAngle = Rotation2d(
            atan2(robotToTargetVector.y, robotToTargetVector.x)
        )

        return targetAngle
    }

    fun followPathToTargetFieldPosition(targetPose: Pose, holdEnd: Boolean, maxPower: Double): Command {
        val currentPose = getPose()
        val poseAsPoint = currentPose.toPoint()

        val pathResult = Weaver.builder()
            .start(poseAsPoint)
            .obstacles(Zones.HIVE_BOTTOM_BAR, Zones.HIVE_TOP_BAR)
            .addTarget(targetPose.toPoint())
            .build()

        return FollowPathCommand(follower, pathResult.toPedroPath(), holdEnd, maxPower)
            .addRequirements(this)
            .beforeStarting(Runnable { syncFollowerToEstimate() })
            .interruptOn { !follower.isBusy }
    }

    fun followPathCMD(path: Path, holdEnd: Boolean, maxPower: Double): Command {
        return FollowPathCommand(follower, path, holdEnd, maxPower)
            .addRequirements(this)
    }

    private fun syncFollowerToEstimate() {
        if (!poseEstimator.isInitialized || isFilterDisabled) return
        follower.setPose(Pose(poseEstimator.x, poseEstimator.y, follower.pose().heading()))
        poseEstimator.resetPose()
    }

    /**
     * Sets a new [Pose] to our robot's chassis.
     * @param pose a pose representing the new robot's [Pose].
     */
    fun setPose(pose: Pose) {
        follower.setPose(pose)
        poseEstimator.resetPose()
    }

    fun configureWeaver() {
        Weaver.setConfig(
            PathConfig()
                .intakeWidth(RobotDimensions.INTAKE_WIDTH.inches / 2.0)
                .robotWidth(RobotDimensions.ROBOT_WIDTH.inches)
                .robotHeight(RobotDimensions.ROBOT_LENGTH.inches)
                .clearance(MecanumConstants.WeaverConfig.CLEARANCE.inches)
        )
    }

    fun getWeaverConfig(): PathConfig {
        return Weaver.getConfig()
    }

    override fun accept(frame: VisionFrame) {
        poseEstimator.addVisionFrame(frame)
    }
}