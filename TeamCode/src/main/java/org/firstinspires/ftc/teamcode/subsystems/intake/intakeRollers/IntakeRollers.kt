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
        intakeRollersMotor = OpMotorEx(hardwareMap, IntakeRollersConstants.Identification.INTAKE_ROLLERS_MOTOR_ID)
        intakeRollersMotor.applyConfigurationAndResetEncoder(IntakeRollersConstants.Configuration.intakeRollersConfiguration)
    }

    override fun periodic() {
        if (intakeRollersMotor.hadVelocityPIDControlGainsUpdated(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_PID)
            || intakeRollersMotor.hadVelocityFeedforwardControlGainsUpdated(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_FEEDFORWARD)) {
            updateIntakeRollersControlGains(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_PID, SubsystemControlGains.INTAKE_ROLLERS_MOTOR_FEEDFORWARD)
        }
    }

    private fun updateIntakeRollersControlGains(pidCoefficients: PIDCoefficients, feedforward: SimpleMotorFeedforward) {
        val newConfig = MotorVelocityModeConfiguration()
            .withVelocityCoefficients(pidCoefficients)
            .withFeedforwardCoefficients(feedforward)

        intakeRollersMotor.applyModeConfiguration(newConfig)
    }

    private fun enableIntakeRollersWithVelocity(velocity: AngularVelocity): Runnable {
        return {
            intakeRollersTargetVelocity = velocity.coerceIn(SubsystemLimits.INTAKE_ROLLERS_MAX_VELOCITY)

            intakeRollersMotor.setVelocity(velocity)
        }
    }

    fun enableIntakeRollersFloorVelocity(): Command {
        return enableIntakeRollersWithVelocity(SubsystemPresetTargets.INTAKE_ROLLERS_FLOOR_RPM).InstantCommand(this)
    }

    fun enableIntakeRollersFlowerVelocity(): Command {
        return enableIntakeRollersWithVelocity(SubsystemPresetTargets.INTAKE_ROLLERS_FLOWER_RPM).InstantCommand(this)
    }

    fun enableIntakeRollersConfigurableVelocity(): Command {
        return enableIntakeRollersWithVelocity(AngularVelocity.fromRpm(SubsystemConfigurableTargets.INTAKE_ROLLERS_CONFIGURABLE_RPM)).InstantCommand(this)
    }

    fun stopIntakeRollers(): Command {
        return intakeRollersMotor.stopMotor().InstantCommand(this)
    }

    fun getIsAtTarget(): Boolean {
        return abs(
            (intakeRollersTargetVelocity.minus(intakeRollersMotor.getVelocity().get())).rpm
        ) < SubsystemTolerances.INTAKE_ROLLERS_RPM_TOLERANCE.rpm
    }

    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Intake Rollers")
        telemetry.addData("Intake Rollers Velocity RPM", intakeRollersMotor.getVelocity().get().rpm)
        telemetry.addData("Intake Rollers Target Velocity RPM", intakeRollersMotor.getVelocity().get().rpm)
    }
}