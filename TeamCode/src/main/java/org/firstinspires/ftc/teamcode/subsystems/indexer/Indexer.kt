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
        indexerMotor = OpMotorEx(hardwareMap, IndexerConstants.Identification.INDEXER_MOTOR_ID)
        indexerMotor.applyConfigurationAndResetEncoder(IndexerConstants.Configuration.indexerConfiguration)
    }

    override fun periodic() {
        if (indexerMotor.hadVelocityPIDControlGainsUpdated(SubsystemControlGains.INDEXER_MOTOR_PID)
            || indexerMotor.hadVelocityFeedforwardControlGainsUpdated(SubsystemControlGains.INDEXER_MOTOR_FEEDFORWARD)) {
            updateIndexerMotorControlGains(SubsystemControlGains.INDEXER_MOTOR_PID, SubsystemControlGains.INDEXER_MOTOR_FEEDFORWARD)
        }
    }

    private fun updateIndexerMotorControlGains(pidCoefficients: PIDCoefficients, feedforward: SimpleMotorFeedforward) {
        val newConfig = MotorVelocityModeConfiguration()
            .withVelocityCoefficients(pidCoefficients)
            .withFeedforwardCoefficients(feedforward)

        indexerMotor.applyModeConfiguration(newConfig)
    }

    private fun enableIndexerWithVelocity(velocity: AngularVelocity): Runnable {
        return {
            indexerTargetVelocity = velocity.coerceIn(SubsystemLimits.INDEXER_MAX_VELOCITY)

            indexerMotor.setVelocity(velocity)
        }
    }

    fun enableIndexerShootingVelocity(): Command {
        return enableIndexerWithVelocity(SubsystemPresetTargets.INDEXER_PRESET_SHOOTING_RPM).InstantCommand(this)
    }

    fun enableIndexerIdleVelocity(): Command {
        return enableIndexerWithVelocity(SubsystemPresetTargets.INDEXER_PRESET_IDLE_RPM).InstantCommand(this)
    }

    fun enableIndexerConfigurableVelocity(): Command {
        return enableIndexerWithVelocity(AngularVelocity.fromRpm(SubsystemConfigurableTargets.INDEXER_CONFIGURABLE_RPM)).InstantCommand(this)
    }

    fun stopIndexer(): Command {
        return indexerMotor.stopMotor().InstantCommand(this)
    }

    fun getIsAtTarget(): Boolean {
        return abs(
            (indexerTargetVelocity.minus(indexerMotor.getVelocity().get())).rpm
        ) < SubsystemTolerances.INDEXER_RPM_TOLERANCE.rpm
    }

    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Indexer")
        telemetry.addData("Indexer Velocity RPM", indexerMotor.getVelocity().get().rpm)
        telemetry.addData("Indexer Target Velocity RPM", indexerTargetVelocity.rpm)
    }
}