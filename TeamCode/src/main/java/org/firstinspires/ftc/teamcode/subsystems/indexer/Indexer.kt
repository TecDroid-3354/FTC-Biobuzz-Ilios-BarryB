package org.firstinspires.ftc.teamcode.subsystems.indexer

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.ftccommon.configuration.EditSwapUsbDevices
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
import org.firstinspires.ftc.teamcode.utils.sensors.ColorSensorConfig
import org.firstinspires.ftc.teamcode.utils.sensors.ColorSensorV3
import org.firstinspires.ftc.teamcode.utils.sensors.DetectedColor
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity
import kotlin.math.abs

class Indexer(private val hardwareMap: HardwareMap): SubsystemBase() {

    private lateinit var indexerMotor: OpMotorEx

    private var indexerTargetVelocity: AngularVelocity = AngularVelocity(0.0)

    private var configurableIndexerVelocity: AngularVelocity = AngularVelocity(0.0)

    private lateinit var topColorSensor: ColorSensorV3; private lateinit var bottomLeftColorSensor: ColorSensorV3; private lateinit var bottomRightColorSensor: ColorSensorV3;

    init {
        // Initialization code
        configureIndexer()
    }

    /** Runs every cycle */
    override fun periodic() {
        // Check whether the PID or Feedforward gains have been changed.
        if (indexerMotor.hadVelocityPIDControlGainsUpdated(SubsystemControlGains.INDEXER_MOTOR_PID)
            || indexerMotor.hadVelocityFeedforwardControlGainsUpdated(SubsystemControlGains.INDEXER_MOTOR_FEEDFORWARD)) {
            updateIndexerMotorControlGains(SubsystemControlGains.INDEXER_MOTOR_PID, SubsystemControlGains.INDEXER_MOTOR_FEEDFORWARD)
        }

        if (configurableIndexerVelocity.rpm.equals(SubsystemConfigurableTargets.INDEXER_CONFIGURABLE_RPM).not()) {
            updateIndexerConfigurableVelocity(SubsystemConfigurableTargets.INDEXER_CONFIGURABLE_RPM)
        }
    }

    /**
     * Updates [configurableIndexerVelocity] with the value of [SubsystemConfigurableTargets.INDEXER_CONFIGURABLE_RPM]
     */
    private fun updateIndexerConfigurableVelocity(newIndexerVelocityRPM: Double) {
        configurableIndexerVelocity = AngularVelocity.fromRpm(newIndexerVelocityRPM)
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
     * Enables the rollers with the [SubsystemPresetTargets.INDEXER_PRESET_SHOOTING_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which enables the rollers at the given velocity.
     */
    fun enableIndexerShootingVelocity(): Runnable {
        return {
            indexerTargetVelocity = SubsystemPresetTargets.INDEXER_PRESET_SHOOTING_RPM

            indexerMotor.setVelocity(SubsystemPresetTargets.INDEXER_PRESET_SHOOTING_RPM)
        }
    }

    /**
     * Enables the rollers with the [SubsystemPresetTargets.INDEXER_PRESET_IDLE_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which enables the rollers with the given velocity.
     */
    fun enableIndexerIdleVelocity(): Runnable {
        return {
            indexerTargetVelocity = SubsystemPresetTargets.INDEXER_PRESET_IDLE_RPM

            indexerMotor.setVelocity(SubsystemPresetTargets.INDEXER_PRESET_IDLE_RPM)
        }
    }

    /**
     * Enables the rollers with the [SubsystemConfigurableTargets.INDEXER_CONFIGURABLE_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which enables the rollers with the given velocity.
     */
    fun enableIndexerConfigurableVelocity(): Runnable {
        return {
            val clampedVelocityRPM = MathUtils.clamp(configurableIndexerVelocity.rpm,
                SubsystemLimits.INDEXER_MAX_VELOCITY.start.rpm, SubsystemLimits.INDEXER_MAX_VELOCITY.endInclusive.rpm)
            indexerTargetVelocity = AngularVelocity.fromRpm(clampedVelocityRPM)

            indexerMotor.setVelocity(configurableIndexerVelocity)
        }
    }

    /**
     * Creates an [com.seattlesolvers.solverslib.command.InstantCommand] based off the [OpMotorEx.stopMotor] method.
     */
    fun stopIndexer(): Command {
        return InstantCommand({
            indexerTargetVelocity = AngularVelocity(0.0)
            indexerMotor.stopMotor()
        }, this)
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

    /** @return the color detected by the top color sensor */
    private fun getTopDetectedColor(): DetectedColor {
        return topColorSensor.getColor()
    }

    /** @return the color detected by the bottom color sensors */
    private fun getBottomDetectedColor(): DetectedColor {
        return if (bottomRightColorSensor.getColor() == bottomLeftColorSensor.getColor()) bottomLeftColorSensor.getColor() else DetectedColor.NO_DETECTION
    }

    /** @return whether the indexer is full */
    fun isIndexerFull(): Boolean {
        var consistencyTracker = 0.0

        return if (getTopDetectedColor() == getBottomDetectedColor()) {
            consistencyTracker++
            consistencyTracker >= IndexerConstants.ColorSensor.REQUIRED_DETECTION_CONSISTENCY
        } else {
            consistencyTracker = 0.0
            false
        }
    }

    // Valuable indexer information, call inside Robot.printTelelmetry()
    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Indexer")
        telemetry.addData("Indexer Velocity RPM", indexerMotor.getVelocity().get().rpm)
        telemetry.addData("Indexer Target Velocity RPM", indexerTargetVelocity.rpm)

        telemetry.addLine("Color Sensors")
        telemetry.addData("Top Color", getTopDetectedColor())
        telemetry.addData("Bottom Color", getBottomDetectedColor())
        telemetry.addData("Is Full", isIndexerFull())
    }

    private fun configureIndexer() {
        indexerMotor = OpMotorEx(hardwareMap, IndexerConstants.Identification.INDEXER_MOTOR_ID)
        indexerMotor.applyConfigurationAndResetEncoder(IndexerConstants.Configuration.indexerConfiguration)

        topColorSensor = ColorSensorV3(hardwareMap, IndexerConstants.ColorSensor.topSensorConfig)
        bottomLeftColorSensor = ColorSensorV3(hardwareMap, IndexerConstants.ColorSensor.bottomLeftSensorConfig)
        bottomRightColorSensor = ColorSensorV3(hardwareMap, IndexerConstants.ColorSensor.bottomRightSensorConfig)
    }
}