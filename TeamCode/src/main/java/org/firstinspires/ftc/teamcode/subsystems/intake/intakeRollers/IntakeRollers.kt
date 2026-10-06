package org.firstinspires.ftc.teamcode.subsystems.intake.intakeRollers

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.devices.OpMotorEx
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.extensions.InstantCommand
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity
import kotlin.math.abs

@Suppress("JoinDeclarationAndAssignment")
class IntakeRollers(hardwareMap: HardwareMap): SubsystemBase() {

    private var intakeRollersMotor: OpMotorEx

    private var intakeRollersTargetVelocity: AngularVelocity = AngularVelocity(0.0)
    
    init {
        // Initialization code
        intakeRollersMotor = OpMotorEx(hardwareMap, IntakeRollersConstants.Identification.INTAKE_ROLLERS_MOTOR_ID)
        intakeRollersMotor.applyConfigurationAndResetEncoder(IntakeRollersConstants.Configuration.intakeRollersConfiguration)
    }

    /** Runs every cycle */
    override fun periodic() {
        if (intakeRollersMotor.hadVelocityPIDControlGainsUpdated(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_PID)
            || intakeRollersMotor.hadVelocityFeedforwardControlGainsUpdated(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_FEEDFORWARD)) {
            updateIntakeRollersControlGains(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_PID, SubsystemControlGains.INTAKE_ROLLERS_MOTOR_FEEDFORWARD)
        }
    }

    /**
     * Updates the [intakeRollersMotor]'s [com.qualcomm.robotcore.hardware.PIDCoefficients] and [SimpleMotorFeedforward]'s coefficients
     * by applying a new configuration.
     * @param pidCoefficients the new [PIDCoefficients]
     * @param feedforward the new [SimpleMotorFeedforward] Coefficients
     */
    private fun updateIntakeRollersControlGains(pidCoefficients: PIDCoefficients, feedforward: SimpleMotorFeedforward) {
        val newConfig = MotorVelocityModeConfiguration()
            .withVelocityCoefficients(pidCoefficients)
            .withFeedforwardCoefficients(feedforward)

        intakeRollersMotor.applyModeConfiguration(newConfig)
    }

    /**
     * A [Runnable] which enables the intake rollers with the given [AngularVelocity] after being clamped
     * by the [SubsystemLimits.INTAKE_ROLLERS_MAX_VELOCITY] limit.
     * @param velocity the desired [AngularVelocity]
     * @return a [Runnable] which enables the rollers
     */
    private fun enableIntakeRollersWithVelocity(velocity: AngularVelocity): Runnable {
        return {
            intakeRollersTargetVelocity = velocity.coerceIn(SubsystemLimits.INTAKE_ROLLERS_MAX_VELOCITY)

            intakeRollersMotor.setVelocity(velocity)
        }
    }

    /**
     * Calls [enableIntakeRollersWithVelocity] and enables the rollers with the [SubsystemPresetTargets.INTAKE_ROLLERS_FLOOR_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which enables the rollers at the given velocity.
     */
    fun enableIntakeRollersFloorVelocity(): Command {
        return enableIntakeRollersWithVelocity(SubsystemPresetTargets.INTAKE_ROLLERS_FLOOR_RPM).InstantCommand(this)
    }

    /**
     * Calls [enableIntakeRollersWithVelocity] and enables the rollers with the [SubsystemPresetTargets.INTAKE_ROLLERS_FLOWER_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which enables the rollers with the given velocity.
     */
    fun enableIntakeRollersFlowerVelocity(): Command {
        return enableIntakeRollersWithVelocity(SubsystemPresetTargets.INTAKE_ROLLERS_FLOWER_RPM).InstantCommand(this)
    }

    /**
     * Calls [enableIntakeRollersWithVelocity] and enables the rollers with the [SubsystemConfigurableTargets.INTAKE_ROLLERS_CONFIGURABLE_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which enables the rollers at the given velocity.
     */
    fun enableIntakeRollersConfigurableVelocity(): Command {
        return enableIntakeRollersWithVelocity(AngularVelocity.fromRpm(SubsystemConfigurableTargets.INTAKE_ROLLERS_CONFIGURABLE_RPM)).InstantCommand(this)
    }

    /**
     * Creates an [com.seattlesolvers.solverslib.command.InstantCommand] based off the [OpMotorEx.stopMotor] method.
     */
    fun stopIntakeRollers(): Command {
        return intakeRollersMotor.stopMotor().InstantCommand(this)
    }

    /**
     * @return whether the intake rollers have reached its target
     */
    fun getIsAtTarget(): Boolean {
        if (intakeRollersTargetVelocity < IntakeRollersConstants.Mechanical.TARGET_VELOCITY_THRESHOLD) return false

        return abs(
            (intakeRollersTargetVelocity.minus(intakeRollersMotor.getVelocity().get())).rpm
        ) < SubsystemTolerances.INTAKE_ROLLERS_RPM_TOLERANCE.rpm
    }

    // Logs useful values
    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Intake Rollers")
        telemetry.addData("Intake Rollers Velocity RPM", intakeRollersMotor.getVelocity().get().rpm)
        telemetry.addData("Intake Rollers Target Velocity RPM", intakeRollersMotor.getVelocity().get().rpm)
    }
}