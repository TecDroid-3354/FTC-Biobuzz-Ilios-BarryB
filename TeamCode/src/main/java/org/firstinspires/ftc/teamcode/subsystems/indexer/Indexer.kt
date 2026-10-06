package org.firstinspires.ftc.teamcode.subsystems.indexer

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import dev.frozenmilk.sinister.loading.Pinned
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

@Pinned
@Suppress("JoinDeclarationAndAssignment")
class Indexer(hardwareMap: HardwareMap): SubsystemBase() {

    private var indexerMotor: OpMotorEx

    private var indexerTargetVelocity: AngularVelocity = AngularVelocity(0.0)

    init {
        // Initialization code
        indexerMotor = OpMotorEx(hardwareMap, IndexerConstants.Identification.INDEXER_MOTOR_ID)
        indexerMotor.applyConfigurationAndResetEncoder(IndexerConstants.Configuration.indexerConfiguration)
    }

    /** Runs every cycle */
    override fun periodic() {
        // Check whether the PID or Feedforward gains have been changed.
        if (indexerMotor.hadVelocityPIDControlGainsUpdated(SubsystemControlGains.INDEXER_MOTOR_PID)
            || indexerMotor.hadVelocityFeedforwardControlGainsUpdated(SubsystemControlGains.INDEXER_MOTOR_FEEDFORWARD)) {
            updateIndexerMotorControlGains(SubsystemControlGains.INDEXER_MOTOR_PID, SubsystemControlGains.INDEXER_MOTOR_FEEDFORWARD)
        }
    }

    /**
     * Updates the [indexerMotor]'s [com.qualcomm.robotcore.hardware.PIDCoefficients] and [SimpleMotorFeedforward]'s coefficients
     * by applying a new configuration.
     * @param pidCoefficients the new [PIDCoefficients]
     * @param feedforward the new [SimpleMotorFeedforward] Coefficients
     */
    private fun updateIndexerMotorControlGains(pidCoefficients: PIDCoefficients, feedforward: SimpleMotorFeedforward) {
        val newConfig = MotorVelocityModeConfiguration()
            .withVelocityCoefficients(pidCoefficients)
            .withFeedforwardCoefficients(feedforward)

        indexerMotor.applyModeConfiguration(newConfig)
    }

    /**
     * A [Runnable] which enables the indexer rollers with the given [AngularVelocity] after being clamped
     * by the [SubsystemLimits.INDEXER_MAX_VELOCITY] limit.
     * @param velocity the desired [AngularVelocity]
     * @return a [Runnable] which enables the rollers
     */
    private fun enableIndexerWithVelocity(velocity: AngularVelocity): Runnable {
        return {
            indexerTargetVelocity = velocity.coerceIn(SubsystemLimits.INDEXER_MAX_VELOCITY)

            indexerMotor.setVelocity(velocity)
        }
    }

    /**
     * Calls [enableIndexerWithVelocity] and enables the rollers with the [SubsystemPresetTargets.INDEXER_PRESET_SHOOTING_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which enables the rollers at the given velocity.
     */
    fun enableIndexerShootingVelocity(): Command {
        return enableIndexerWithVelocity(SubsystemPresetTargets.INDEXER_PRESET_SHOOTING_RPM).InstantCommand(this)
    }

    /**
     * Calls [enableIndexerWithVelocity] and enables the rollers with the [SubsystemPresetTargets.INDEXER_PRESET_IDLE_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which enables the rollers with the given velocity.
     */
    fun enableIndexerIdleVelocity(): Command {
        return enableIndexerWithVelocity(SubsystemPresetTargets.INDEXER_PRESET_IDLE_RPM).InstantCommand(this)
    }

    /**
     * Calls [enableIndexerWithVelocity] and enables the rollers with the [SubsystemConfigurableTargets.INDEXER_CONFIGURABLE_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which enables the rollers with the given velocity.
     */
    fun enableIndexerConfigurableVelocity(): Command {
        return enableIndexerWithVelocity(AngularVelocity.fromRpm(SubsystemConfigurableTargets.INDEXER_CONFIGURABLE_RPM)).InstantCommand(this)
    }

    /**
     * Creates an [com.seattlesolvers.solverslib.command.InstantCommand] based off the [OpMotorEx.stopMotor] method.
     */
    fun stopIndexer(): Command {
        return indexerMotor.stopMotor().InstantCommand(this)
    }

    /**
     * @return whether the indexer has reached its [indexerTargetVelocity]
     */
    fun getIsAtTarget(): Boolean {
        if (indexerTargetVelocity < IndexerConstants.Mechanical.TARGET_VELOCITY_THRESHOLD) return false

        return abs(
            (indexerTargetVelocity.minus(indexerMotor.getVelocity().get())).rpm
        ) < SubsystemTolerances.INDEXER_RPM_TOLERANCE.rpm
    }

    // Valuable indexer information, call inside Robot.printTelelmetry()
    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Indexer")
        telemetry.addData("Indexer Velocity RPM", indexerMotor.getVelocity().get().rpm)
        telemetry.addData("Indexer Target Velocity RPM", indexerTargetVelocity.rpm)
    }
}