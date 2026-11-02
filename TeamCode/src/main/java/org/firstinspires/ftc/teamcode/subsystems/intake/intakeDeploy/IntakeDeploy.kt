package org.firstinspires.ftc.teamcode.subsystems.intake.intakeDeploy

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx
import org.firstinspires.ftc.teamcode.utils.devices.RTPServo
import org.firstinspires.ftc.teamcode.utils.devices.RTPServoConfig
import org.firstinspires.ftc.teamcode.utils.extensions.InstantCommand
import org.firstinspires.ftc.teamcode.utils.units.Angle
import kotlin.math.abs

class IntakeDeploy(private val hardwareMap: HardwareMap, private val telemetry: TelemetryManager): SubsystemBase() {

    private lateinit var leadIntakeDeployServo: RTPServo

    private lateinit var followerIntakeDeployServo: RTPServo

    private var intakeDeployTargetAngle: Angle = Angle.fromDegrees(0.0)

    private var configurableIntakeDeployAngle: Angle = Angle(0.0)

    init {
        // Initialization code
        configureServos()

        setIntakeDeployFloorAngle().run()
    }

    /** Runs every cycle */
    override fun periodic() {
        leadIntakeDeployServo.periodic()
        followerIntakeDeployServo.periodic()

        if (leadIntakeDeployServo.hadPIDFCoefficientsUpdated(SubsystemControlGains.INTAKE_DEPLOY_SERVOS_PIDF)) {
            updateIntakeDeployPIDF(SubsystemControlGains.INTAKE_DEPLOY_SERVOS_PIDF)
        }

        if (configurableIntakeDeployAngle.degrees.equals(SubsystemConfigurableTargets.INTAKE_DEPLOY_CONFIGURABLE_DEGREES).not()) {
            updateIntakeDeployConfigurableAngle(SubsystemConfigurableTargets.INTAKE_DEPLOY_CONFIGURABLE_DEGREES)
        }
    }

    /** Updates [configurableIntakeDeployAngle] with the value of [SubsystemConfigurableTargets.INTAKE_DEPLOY_CONFIGURABLE_DEGREES]*/
    private fun updateIntakeDeployConfigurableAngle(newIntakeDeployConfigurableAngleDegrees: Double) {
        configurableIntakeDeployAngle = Angle.fromDegrees(newIntakeDeployConfigurableAngleDegrees)
    }

    /**
     * Updates the [leadIntakeDeployServo] and [followerIntakeDeployServo]s [com.qualcomm.robotcore.hardware.PIDCoefficients] coefficients
     * by calling [OpServoEx.updateRunToPositionPIDF]
     * @param pidfCoefficients the new [PIDCoefficients]
     */
    private fun updateIntakeDeployPIDF(pidfCoefficients: PIDFCoefficients) {
        leadIntakeDeployServo.setPIDF(pidfCoefficients)
        followerIntakeDeployServo.setPIDF(pidfCoefficients)
    }

    /**
     * Sets the floor intake angle to [SubsystemPresetTargets.INTAKE_DEPLOY_FLOOR_ANGLE] value.
     * @return a [Runnable] which sets a new angle.
     */
    fun setIntakeDeployFloorAngle(): Runnable {
        return {
            intakeDeployTargetAngle = SubsystemPresetTargets.INTAKE_DEPLOY_FLOOR_ANGLE

            leadIntakeDeployServo.setTargetAngle(SubsystemPresetTargets.INTAKE_DEPLOY_FLOOR_ANGLE)
            followerIntakeDeployServo.setTargetAngle(SubsystemPresetTargets.INTAKE_DEPLOY_FLOOR_ANGLE)
        }
    }

    /**
     * Sets the flower intake angle to [SubsystemPresetTargets.INTAKE_DEPLOY_FLOWER_ANGLE] value.
     * @return a [Runnable] which sets a new angle.
     */
    fun setIntakeDeployFlowerAngle(): Runnable {
        return {
            intakeDeployTargetAngle = SubsystemPresetTargets.INTAKE_DEPLOY_FLOWER_ANGLE

            leadIntakeDeployServo.setTargetAngle(SubsystemPresetTargets.INTAKE_DEPLOY_FLOWER_ANGLE)
            followerIntakeDeployServo.setTargetAngle(SubsystemPresetTargets.INTAKE_DEPLOY_FLOWER_ANGLE)
        }
    }

    /**
     * Sets the intake deploy angle to [SubsystemConfigurableTargets.INTAKE_DEPLOY_CONFIGURABLE_DEGREES] value.
     * @return a [Runnable] which sets a new angle.
     */
    fun setIntakeDeployConfigurableAngle(): Runnable {
        return {
            val clampedAngleDegrees = MathUtils.clamp(configurableIntakeDeployAngle.degrees,
                SubsystemLimits.INTAKE_DEPLOY_ANGLE_LIMITS.start.degrees, SubsystemLimits.INTAKE_DEPLOY_ANGLE_LIMITS.endInclusive.degrees)
            intakeDeployTargetAngle = Angle.fromDegrees(clampedAngleDegrees)

            leadIntakeDeployServo.setTargetAngle(intakeDeployTargetAngle)
            followerIntakeDeployServo.setTargetAngle(intakeDeployTargetAngle)
        }
    }

    /**
     * @return whether the [IntakeDeploy] has reached its angle target.
     */
    fun getIsAtTarget(): Boolean {
        if (intakeDeployTargetAngle < IntakeDeployConstants.Mechanical.TARGET_ANGLE_THRESHOLD) return false

        return abs(
            (intakeDeployTargetAngle.minus(leadIntakeDeployServo.getAngle())).degrees
        ) < SubsystemTolerances.INTAKE_DEPLOY_ANGLE_TOLERANCE.degrees
    }

    // Valuable indexer information, call inside Robot.printTelelmetry()
    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Intake Deploy")
        telemetry.addData("Intake Deploy Lead Servo Angle Degrees", leadIntakeDeployServo.getAngle().degrees)
        telemetry.addData("Intake Deploy Follower Servo Angle Degrees", followerIntakeDeployServo.getAngle().degrees)
        telemetry.addData("Intake Deploy Target Angle Degrees", intakeDeployTargetAngle.degrees)
    }

    // Initialize both servos
    private fun configureServos() {
        followerIntakeDeployServo = RTPServo(hardwareMap,
            RTPServoConfig(
                IntakeDeployConstants.Identification.INTAKE_DEPLOY_FOLLOWER_SERVO_ID,
                IntakeDeployConstants.Identification.INTAKE_DEPLOY_FOLLOWER_SERVO_ABSOLUTE_ID,
                IntakeDeployConstants.Configuration.followerAbsoluteMaxVoltage,
                RTPServo.Direction.FORWARD,
                IntakeDeployConstants.Mechanical.followerAbsoluteEncoderOffset,
                1.0,
                IntakeDeployConstants.Mechanical.GEAR_RATIO,
                SubsystemControlGains.INTAKE_DEPLOY_SERVOS_PIDF
            ))

        leadIntakeDeployServo = RTPServo(hardwareMap,
            RTPServoConfig(
                IntakeDeployConstants.Identification.INTAKE_DEPLOY_LEAD_SERVO_ID,
                IntakeDeployConstants.Identification.INTAKE_DEPLOY_LEAD_SERVO_ABSOLUTE_ID,
                IntakeDeployConstants.Configuration.leadAbsoluteMaxVoltage,
                RTPServo.Direction.REVERSE,
                IntakeDeployConstants.Mechanical.leadAbsoluteEncoderOffset,
                1.0,
                IntakeDeployConstants.Mechanical.GEAR_RATIO,
                SubsystemControlGains.INTAKE_DEPLOY_SERVOS_PIDF
            ))
    }
}