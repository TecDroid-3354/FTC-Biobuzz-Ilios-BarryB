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
import org.firstinspires.ftc.teamcode.utils.extensions.InstantCommand
import org.firstinspires.ftc.teamcode.utils.units.Angle
import kotlin.math.abs

class IntakeDeploy(private val hardwareMap: HardwareMap): SubsystemBase() {

    private lateinit var leadIntakeDeployServo: OpServoEx

    private lateinit var followerIntakeDeployServo: OpServoEx

    private var intakeDeployTargetAngle: Angle = Angle(0.0)

    private var configurableIntakeDeployAngle: Angle = Angle(0.0)

    init {
        // Initialization code
        configureServos()
    }

    /** Runs every cycle */
    override fun periodic() {
        if (leadIntakeDeployServo.hadRTPCoefficientsUpdated(SubsystemControlGains.INTAKE_DEPLOY_SERVOS_PIDF)) {
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
        leadIntakeDeployServo.updateRunToPositionPIDF(pidfCoefficients)
        followerIntakeDeployServo.updateRunToPositionPIDF(pidfCoefficients)
    }

    /**
     * Sets the floor intake angle to [SubsystemPresetTargets.INTAKE_DEPLOY_FLOOR_ANGLE] value.
     * @return a [Runnable] which sets a new angle.
     */
    fun setIntakeDeployFloorAngle(): Runnable {
        return {
            intakeDeployTargetAngle = SubsystemPresetTargets.INTAKE_DEPLOY_FLOOR_ANGLE

            leadIntakeDeployServo.runToPosition(SubsystemPresetTargets.INTAKE_DEPLOY_FLOOR_ANGLE)
            followerIntakeDeployServo.runToPosition(SubsystemPresetTargets.INTAKE_DEPLOY_FLOOR_ANGLE)
        }
    }

    /**
     * Sets the flower intake angle to [SubsystemPresetTargets.INTAKE_DEPLOY_FLOWER_ANGLE] value.
     * @return a [Runnable] which sets a new angle.
     */
    fun setIntakeDeployFlowerAngle(): Runnable {
        return {
            intakeDeployTargetAngle = SubsystemPresetTargets.INTAKE_DEPLOY_FLOWER_ANGLE

            leadIntakeDeployServo.runToPosition(SubsystemPresetTargets.INTAKE_DEPLOY_FLOWER_ANGLE)
            followerIntakeDeployServo.runToPosition(SubsystemPresetTargets.INTAKE_DEPLOY_FLOWER_ANGLE)
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

            leadIntakeDeployServo.runToPosition(intakeDeployTargetAngle)
            followerIntakeDeployServo.runToPosition(intakeDeployTargetAngle)
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
        leadIntakeDeployServo = OpServoEx(hardwareMap, IntakeDeployConstants.Identification.INTAKE_DEPLOY_LEAD_SERVO_ID)
        leadIntakeDeployServo.applyConfiguration(IntakeDeployConstants.Configuration.leadServoConfiguration)

        followerIntakeDeployServo = OpServoEx(hardwareMap, IntakeDeployConstants.Identification.INTAKE_DEPLOY_FOLLOWER_SERVO_ID)
        followerIntakeDeployServo.applyConfiguration(IntakeDeployConstants.Configuration.followerServoConfiguration)
    }
}