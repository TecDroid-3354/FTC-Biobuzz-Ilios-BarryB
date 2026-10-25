package org.firstinspires.ftc.teamcode.subsystems.intake.intakeRollers

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.devices.OpMotorEx
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity
import kotlin.math.abs

class IntakeRollers(private val hardwareMap: HardwareMap): SubsystemBase() {

    private lateinit var leadIntakeRollersMotor: OpMotorEx
    private lateinit var followerIntakeRollersMotor: OpMotorEx

    private var intakeRollersTargetVelocity: AngularVelocity = AngularVelocity(0.0)

    private var configurableIntakeRollersVelocity: AngularVelocity = AngularVelocity(0.0)
    
    init {
        // Initialization code
        configureMotors()
    }

    /** Runs every cycle */
    override fun periodic() {
        if (leadIntakeRollersMotor.hadVelocityPIDControlGainsUpdated(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_PID)
            || leadIntakeRollersMotor.hadVelocityFeedforwardControlGainsUpdated(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_FEEDFORWARD)) {
            updateIntakeRollersControlGains(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_PID, SubsystemControlGains.INTAKE_ROLLERS_MOTOR_FEEDFORWARD)
        }

        if (configurableIntakeRollersVelocity.rpm.equals(SubsystemConfigurableTargets.INTAKE_ROLLERS_CONFIGURABLE_RPM).not()) {
            updateIntakeRollersConfigurableVelocity(SubsystemConfigurableTargets.INTAKE_ROLLERS_CONFIGURABLE_RPM)
        }
    }

    /** Updates [configurableIntakeRollersVelocity] with the value of [SubsystemConfigurableTargets.INTAKE_ROLLERS_CONFIGURABLE_RPM] */
    private fun updateIntakeRollersConfigurableVelocity(newIntakeRollersVelocityRPM: Double) {
        configurableIntakeRollersVelocity = AngularVelocity.fromRpm(newIntakeRollersVelocityRPM)
    }

    /**
     * Updates the [leadIntakeRollersMotor]'s [com.qualcomm.robotcore.hardware.PIDCoefficients] and [SimpleMotorFeedforward]'s coefficients
     * by applying a new configuration.
     * @param pidCoefficients the new [PIDCoefficients]
     * @param feedforward the new [SimpleMotorFeedforward] Coefficients
     */
    private fun updateIntakeRollersControlGains(pidCoefficients: PIDCoefficients, feedforward: SimpleMotorFeedforward) {
        val newConfig = MotorVelocityModeConfiguration()
            .withVelocityCoefficients(pidCoefficients)
            .withFeedforwardCoefficients(feedforward)

        leadIntakeRollersMotor.applyModeConfiguration(newConfig)
    }

    /**
     * Enables the rollers with the [SubsystemPresetTargets.INTAKE_ROLLERS_FLOOR_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which enables the rollers at the given velocity.
     */
    fun enableIntakeRollersFloorVelocity(): Runnable {
        return {
            intakeRollersTargetVelocity = SubsystemPresetTargets.INTAKE_ROLLERS_FLOOR_RPM

            leadIntakeRollersMotor.setVelocity(SubsystemPresetTargets.INTAKE_ROLLERS_FLOOR_RPM)
            followerIntakeRollersMotor.setVelocity(SubsystemPresetTargets.INTAKE_ROLLERS_FLOOR_RPM)
        }
    }

    /**
     * Enables the rollers with the [SubsystemPresetTargets.INTAKE_ROLLERS_FLOWER_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which enables the rollers with the given velocity.
     */
    fun enableIntakeRollersFlowerVelocity(): Runnable {
        return {
            intakeRollersTargetVelocity = SubsystemPresetTargets.INTAKE_ROLLERS_FLOWER_RPM

            leadIntakeRollersMotor.setVelocity(SubsystemPresetTargets.INTAKE_ROLLERS_FLOWER_RPM)
            followerIntakeRollersMotor.setVelocity(SubsystemPresetTargets.INTAKE_ROLLERS_FLOWER_RPM)
        }
    }

    /**
     * Enables the rollers with the [SubsystemConfigurableTargets.INTAKE_ROLLERS_CONFIGURABLE_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which enables the rollers at the given velocity.
     */
    fun enableIntakeRollersConfigurableVelocity(): Runnable {
        return {
            val clampedVelocityRPM = MathUtils.clamp(configurableIntakeRollersVelocity.rpm,
                SubsystemLimits.INTAKE_ROLLERS_MAX_VELOCITY.start.rpm, SubsystemLimits.INTAKE_ROLLERS_MAX_VELOCITY.endInclusive.rpm)
            intakeRollersTargetVelocity = AngularVelocity.fromRpm(clampedVelocityRPM)

            leadIntakeRollersMotor.setVelocity(configurableIntakeRollersVelocity)
            followerIntakeRollersMotor.setVelocity(configurableIntakeRollersVelocity)
        }
    }

    /**
     * Creates an [com.seattlesolvers.solverslib.command.InstantCommand] based off the [OpMotorEx.stopMotor] method.
     */
    fun stopIntakeRollers(): Command {
        return InstantCommand({
            intakeRollersTargetVelocity = AngularVelocity(0.0)
            leadIntakeRollersMotor.stopMotor()
            followerIntakeRollersMotor.stopMotor()
        }, this)
    }

    /** @return whether the intake rollers have reached its target */
    fun getIsAtTarget(): Boolean {
        if (intakeRollersTargetVelocity < IntakeRollersConstants.Mechanical.TARGET_VELOCITY_THRESHOLD) return false

        return abs(
            (intakeRollersTargetVelocity.minus(leadIntakeRollersMotor.getVelocity().get())).rpm
        ) < SubsystemTolerances.INTAKE_ROLLERS_RPM_TOLERANCE.rpm
    }

    // Logs useful values
    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Intake Rollers")
        telemetry.addData("Intake Rollers Velocity RPM", leadIntakeRollersMotor.getVelocity().get().rpm)
        telemetry.addData("Intake Rollers Target Velocity RPM", leadIntakeRollersMotor.getVelocity().get().rpm)
    }

    private fun configureMotors() {
        leadIntakeRollersMotor = OpMotorEx(hardwareMap, IntakeRollersConstants.Identification.LEAD_INTAKE_ROLLERS_MOTOR_ID)
        leadIntakeRollersMotor.applyConfigurationAndResetEncoder(IntakeRollersConstants.Configuration.leadMotorConfiguration)

        followerIntakeRollersMotor = OpMotorEx(hardwareMap, IntakeRollersConstants.Identification.LEAD_INTAKE_ROLLERS_MOTOR_ID)
        followerIntakeRollersMotor.applyConfigurationAndResetEncoder(IntakeRollersConstants.Configuration.followerMotorConfiguration)
    }
}