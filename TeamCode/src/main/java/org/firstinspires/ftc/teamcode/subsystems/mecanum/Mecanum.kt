package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.pedropathing.api.Paths
import com.pedropathing.follower.Follower
import com.pedropathing.follower.ManualDrive
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.geometry.Transform2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.seattlesolvers.solverslib.geometry.Vector2d
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.constants.DriveMultipliers
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.extensions.toPedroPose
import org.firstinspires.ftc.teamcode.utils.extensions.toPose2d
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.teamcode.utils.units.LinearVelocity
import java.util.Optional
import kotlin.math.atan2
import kotlin.math.hypot

class Mecanum(
    private val follower: Follower,
    private val controller: GamepadEx,
    private val alliance: Alliance
): SubsystemBase(), Limelight.VisionConsumer {

    /**
     * Runs in every loop. Follower and telemetry get updated
     */
    override fun periodic() {
        follower.update()
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

    /**
     * Gets the Follower's current position.
     * @return a [Pose2D] containing the robot's current position in the standard FTC Coordinates
     */
    fun getPose(): Pose2d {
        return follower.pose().toPose2d()
    }

    /**
     * Gets the [Follower]'s rotation component.
     * @return a [Rotation2d] as the robot's current heading in radians.
     */
    fun getRotation(): Rotation2d {
        return Rotation2d(getPose().heading)
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
        val robotToTargetVector = fieldToTarget.minus(getPose().translation)
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

        val robotToTargetVector = fieldToTarget.minus(getPose().translation)
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
        val robotToTargetVector = fieldToTarget.minus(getPose().translation)

        val targetAngle = Rotation2d(
            atan2(robotToTargetVector.y, robotToTargetVector.x)
        )

        return targetAngle
    }

    fun followPathToTargetFieldPosition(targetPose: Pose2d, holdEnd: Boolean, maxPower: Double): Command {
        // TODO Check if the Instant Command works, if not, change to Run Command
        val currentPosePedro = getPose().toPedroPose()
        val targetPosePedro = targetPose.toPedroPose()
        val targetHeadingPedro = Pose(currentPosePedro.x(), currentPosePedro.y(),targetPosePedro.heading())

        val path = Paths.curve(currentPosePedro, targetHeadingPedro, targetPosePedro)

        return FollowPathCommand(follower, path, holdEnd, maxPower)
            .addRequirements(this)
            .beforeStarting(Runnable { follower.stop() })
    }

    fun followPathCMD(path: Path, holdEnd: Boolean, maxPower: Double): Command {
        return FollowPathCommand(follower, path, holdEnd, maxPower)
            .addRequirements(this)
    }

    /**
     * Sets a new [Pose] to our robot's chassis.
     * @param pose a pose representing the new robot's [Pose].
     */
    fun setPose(pose: Pose) {
        follower.setPose(pose)
    }

    override fun accept(estimatedPose: Pose) {
        val currentPose = getPose().toPedroPose()

        setPose(
            Pose(
                currentPose.x() + MecanumConstants.VisionPoseBlends.POSE_BLEND * (estimatedPose.x().minus(currentPose.x())),
                currentPose.y() + MecanumConstants.VisionPoseBlends.POSE_BLEND * (estimatedPose.y().minus(currentPose.y())),
                currentPose.heading() + MecanumConstants.VisionPoseBlends.HEADING_BLEND * AngleUnit.normalizeRadians(estimatedPose.heading().minus(currentPose.heading()))
            )
        )
    }
}