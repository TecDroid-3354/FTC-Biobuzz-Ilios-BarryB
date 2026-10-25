package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.util.Debouncer
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.devices.OpMotorEx
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.teamcode.utils.units.Time
import java.util.function.Supplier
import kotlin.collections.iterator
import kotlin.math.abs

class Flywheel(private val hardwareMap: HardwareMap): SubsystemBase() {

    private lateinit var flywheelMotor: OpMotorEx

    private var flywheelTargetVelocity: AngularVelocity = AngularVelocity(0.0)

    private var configurableFlywheelTargetVelocity: AngularVelocity = AngularVelocity(0.0)

    private val atTargetDebouncer: Debouncer = Debouncer(0.06, Debouncer.DebounceType.Both)

    init {
        // Initialization code
        configureMotors()
    }

    /** Runs every cycle */
    override fun periodic() {
        if (flywheelMotor.hadVelocityPIDControlGainsUpdated(SubsystemControlGains.FLYWHEEL_MOTOR_PID)
            || flywheelMotor.hadVelocityFeedforwardControlGainsUpdated(SubsystemControlGains.FLYWHEEL_MOTOR_FEEDFORWARD)) {
            updateFlywheelMotorControlGains(SubsystemControlGains.FLYWHEEL_MOTOR_PID, SubsystemControlGains.FLYWHEEL_MOTOR_FEEDFORWARD)
        }

        if (configurableFlywheelTargetVelocity.rpm.equals(SubsystemConfigurableTargets.FLYWHEEL_CONFIGURABLE_RPM).not()) {
            updateFlywheelConfigurableVelocity(SubsystemConfigurableTargets.FLYWHEEL_CONFIGURABLE_RPM)
        }
    }

    /**
     * Update [configurableFlywheelTargetVelocity] with [SubsystemConfigurableTargets.FLYWHEEL_CONFIGURABLE_RPM]
     */
    private fun updateFlywheelConfigurableVelocity(newFlywheelConfigurableVelocityRPM: Double) {
        configurableFlywheelTargetVelocity = AngularVelocity.fromRpm(newFlywheelConfigurableVelocityRPM)
    }

    /**
     * Updates the [flywheelMotor]'s [com.qualcomm.robotcore.hardware.PIDCoefficients] and [SimpleMotorFeedforward]'s coefficients
     * by applying a new configuration.
     * @param pidCoefficients the new [PIDCoefficients]
     * @param feedforward the new [SimpleMotorFeedforward] Coefficients
     */
    private fun updateFlywheelMotorControlGains(pidCoefficients: PIDCoefficients, feedforward: SimpleMotorFeedforward) {
        val newConfig = MotorVelocityModeConfiguration()
            .withVelocityCoefficients(pidCoefficients)
            .withFeedforwardCoefficients(feedforward)

        flywheelMotor.applyModeConfiguration(newConfig)
    }

    /**
     * Enables the flywheel with the [SubsystemPresetTargets.FLYWHEEL_PRESET_RPM] value.
     * @return an [Runnable] which enables the flywheel at the given velocity.
     */
    fun setFlywheelPresetVelocity(): Runnable {
        return {
            flywheelTargetVelocity = SubsystemPresetTargets.FLYWHEEL_PRESET_RPM

            flywheelMotor.setVelocity(SubsystemPresetTargets.FLYWHEEL_PRESET_RPM)
        }
    }

    /**
     * Enables the flywheel with the [SubsystemConfigurableTargets.FLYWHEEL_CONFIGURABLE_RPM] value.
     * @return an [Runnable] which enables the rollers with the given velocity.
     */
    fun setFlywheelConfigurableVelocity(): Runnable {
        return {
            val clampedVelocityRPM = MathUtils.clamp(configurableFlywheelTargetVelocity.rpm,
                SubsystemLimits.FLYWHEEL_MAX_VELOCITY.start.rpm, SubsystemLimits.FLYWHEEL_MAX_VELOCITY.endInclusive.rpm)
            flywheelTargetVelocity = AngularVelocity.fromRpm(clampedVelocityRPM)

            flywheelMotor.setVelocity(configurableFlywheelTargetVelocity)
        }
    }

    /**
     * Enables the flywheel at [flywheelCalculatedVelocity] and the [FlywheelConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT].
     * @return a [Runnable] that enables the flywheel with the calculated velocity.
     */
    private fun setFlywheelCalculatedScoringVelocity(flywheelCalculatedVelocity: AngularVelocity): Runnable {
        return {
            val clampedVelocityRPM = MathUtils.clamp(flywheelCalculatedVelocity.rpm,
                SubsystemLimits.FLYWHEEL_MAX_VELOCITY.start.rpm, SubsystemLimits.FLYWHEEL_MAX_VELOCITY.endInclusive.rpm)
            flywheelTargetVelocity = AngularVelocity.fromRpm(clampedVelocityRPM)

            flywheelMotor.setVelocity(flywheelCalculatedVelocity)
        }
    }

    /**
     * Creates a [RunCommand] based off [setFlywheelCalculatedScoringVelocity] which updates the [flywheelDistanceToTarget]
     * for each loop.
     * @return a [RunCommand] which enables the flywheel calculated velocity
     */
    fun setFlywheelCalculatedScoringVelocity(flywheelDistanceToTarget: Supplier<Distance>): Command {
        return RunCommand({
            val flywheelCalculatedVelocity = getCalculatedHiveScoringVelocity(flywheelDistanceToTarget.get())
            setFlywheelCalculatedScoringVelocity(flywheelCalculatedVelocity).run()
        }, this)
    }

    /**
     * Creates an [com.seattlesolvers.solverslib.command.InstantCommand] based off the [OpMotorEx.stopMotor] method.
     */
    fun stopFlywheel(): Command {
        return InstantCommand({
            flywheelTargetVelocity = AngularVelocity(0.0)
            flywheelMotor.stopMotor()
        }, this)
    }

    /**
     * Calculate the HIVE Scoring [Flywheel] target [AngularVelocity] based on the distance to its target.
     */
    private fun getCalculatedHiveScoringVelocity(flywheelDistanceToTarget: Distance): AngularVelocity {
        val distanceInMeters = flywheelDistanceToTarget.meters
        val calculatedRPMs = FlywheelConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT.get(distanceInMeters)

        return AngularVelocity.fromRpm(calculatedRPMs)
    }

    /**
     * Calculate the Scoring TOF [Flywheel] target [AngularVelocity] based on the distance to its target.
     */
    fun getCalculatedScoringTimeOfFlight(flywheelDistanceToTarget: Distance): Time {
        val distanceInMeters = flywheelDistanceToTarget.meters
        val calculatedTOF = FlywheelConstants.Interpolation.TIME_OF_FLIGHT_HIVE_INTERPOLATED_LUT.get(distanceInMeters)

        return Time(calculatedTOF)
    }

    /**
     * @return whether the flywheel has reached its target
     */
    fun getIsAtTarget(): Boolean {
        if (flywheelTargetVelocity < FlywheelConstants.Mechanical.VELOCITY_TARGET_THRESHOLD) return false

        val isWithinTolerance =  abs(
            (flywheelTargetVelocity.minus(flywheelMotor.getVelocity().get())).rpm
        ) < SubsystemTolerances.FLYWHEEL_RPM_TOLERANCE.rpm

        return atTargetDebouncer.calculate(isWithinTolerance)
    }

    // Logs useful values
    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Flywheel")
        telemetry.addData("Flywheel Lead Motor Connected", flywheelMotor.getIsConnected().asBoolean)
        telemetry.addData("Flywheel Target Velocity RPM", flywheelTargetVelocity.rpm)
        telemetry.addData("Flywheel Velocity RPM", flywheelMotor.getVelocity().get().rpm)
    }

    // Motor and interpolation configuration
    private fun configureMotors() {
        flywheelMotor = OpMotorEx(hardwareMap, FlywheelConstants.Identification.FLYWHEEL_LEAD_MOTOR_ID)
        flywheelMotor.applyConfigurationAndResetEncoder(FlywheelConstants.Configuration.leadMotorConfiguration)

        for (point in FlywheelConstants.Interpolation.SCORING_POINTS_LIST) {
            FlywheelConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT.add(point.key.meters, point.value.rpm)
        }

        FlywheelConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT.createLUT()

        for (point in FlywheelConstants.Interpolation.TIME_OF_FLIGHT_POINTS_LIST) {
            FlywheelConstants.Interpolation.TIME_OF_FLIGHT_HIVE_INTERPOLATED_LUT.add(point.key.meters, point.value.seconds)
        }

        FlywheelConstants.Interpolation.TIME_OF_FLIGHT_HIVE_INTERPOLATED_LUT.createLUT()
    }
}