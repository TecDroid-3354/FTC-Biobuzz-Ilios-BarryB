package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.util.Debouncer
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains.FLYWHEEL_MOTOR_FEEDFORWARD
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.controllers.BangBangController
import org.firstinspires.ftc.teamcode.utils.devices.OpMotorEx
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.teamcode.utils.units.Time
import org.firstinspires.ftc.teamcode.utils.units.Voltage
import java.util.function.Supplier
import kotlin.math.abs

class Flywheel(private val hardwareMap: HardwareMap, private val voltage: Supplier<Voltage>): SubsystemBase() {

    private companion object {
        /**
         * Fraction of the feedforward applied alongside the bang-bang output.
         * Slightly under 1.0 (WPILib recommends 0.9) so the feedforward alone never
         * pushes the flywheel past the setpoint, letting bang-bang handle the final approach.
         */
        const val FEEDFORWARD_SCALE = 0.9
    }

    private lateinit var flywheelMotor: OpMotorEx

    private var flywheelTargetVelocity: AngularVelocity = AngularVelocity()

    private var configurableFlywheelTargetVelocity: AngularVelocity = AngularVelocity()

    private val bangBangController: BangBangController = BangBangController()

    private var flywheelFeedforward: SimpleMotorFeedforward = FLYWHEEL_MOTOR_FEEDFORWARD

    private val atTargetDebouncer: Debouncer = Debouncer(0.06, Debouncer.DebounceType.Both)

    init {
        // Initialization code
        configureMotors()
    }

    /** Runs every cycle */
    override fun periodic() {
        // Update Feedforward
        if (flywheelFeedforward.ks != FLYWHEEL_MOTOR_FEEDFORWARD.ks
            || flywheelFeedforward.kv != FLYWHEEL_MOTOR_FEEDFORWARD.kv) {
            flywheelFeedforward = SimpleMotorFeedforward(FLYWHEEL_MOTOR_FEEDFORWARD.ks, FLYWHEEL_MOTOR_FEEDFORWARD.kv)
        }

        // Update configurable velocity
        if (configurableFlywheelTargetVelocity.rpm.equals(SubsystemConfigurableTargets.FLYWHEEL_CONFIGURABLE_RPM).not()) {
            updateFlywheelConfigurableVelocity(SubsystemConfigurableTargets.FLYWHEEL_CONFIGURABLE_RPM)
        }

        // Motor is mounted reversed, so the measured velocity is negated to be positive when spinning up.
        val measuredRpm = flywheelMotor.getVelocity().get().rpm
        val targetRpm = flywheelTargetVelocity.rpm

        // Always update the controller so its measurement/setpoint (and atSetpoint()) stay current.
        // Returns 1.0 when below the setpoint, otherwise 0.0.
        val bangBangOutput = bangBangController.calculate(measuredRpm, targetRpm)

        // Don't move if the target is not greater than 0.
        if (targetRpm > 0.0) {
            // Feedforward holds the flywheel near the setpoint, scaled so it never overshoots on its own.
            val feedforward = flywheelFeedforward.calculate(targetRpm) * FEEDFORWARD_SCALE
            // Compensate only the feedforward for battery sag (bang-bang is already full power).
            val voltageFactor = 12.0 / voltage.get().volts
            // Bang-bang (0 or 1) + compensated feedforward, never negative so the motor never brakes or reverses.
            val power = MathUtils.clamp((bangBangOutput * voltageFactor) + feedforward , 0.0, 1.0)

            flywheelMotor.setPower(power)
        } else {
            flywheelMotor.setPower(0.0)
        }
    }

    /**
     * Update [configurableFlywheelTargetVelocity] with [SubsystemConfigurableTargets.FLYWHEEL_CONFIGURABLE_RPM]
     */
    private fun updateFlywheelConfigurableVelocity(newFlywheelConfigurableVelocityRPM: Double) {
        configurableFlywheelTargetVelocity = AngularVelocity.fromRpm(newFlywheelConfigurableVelocityRPM)
    }

    /**
     * Enables the flywheel with the [SubsystemPresetTargets.FLYWHEEL_IDLE_RPM] value.
     * @return an [Runnable] which enables the flywheel at the given velocity.
     */
    fun setFlywheelIdleVelocity(): Runnable {
        return {
            flywheelTargetVelocity = SubsystemPresetTargets.FLYWHEEL_IDLE_RPM
        }
    }

    /**
     * Enables the flywheel with the [SubsystemPresetTargets.FLYWHEEL_PRESET_SHOOTING_RPM] value.
     * @return an [Runnable] which enables the flywheel at the given velocity.
     */
    fun setFlywheelPresetShootingVelocity(): Runnable {
        return {
            flywheelTargetVelocity = SubsystemPresetTargets.FLYWHEEL_PRESET_SHOOTING_RPM
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
     * Calculate the HIVE Scoring [Flywheel] target [AngularVelocity] based on the distance to its target
     * And making use of [FlywheelConstants.StoredPolynomials.scoringPolynomial].
     */
    private fun getCalculatedHiveScoringVelocity(flywheelDistanceToTarget: Distance): AngularVelocity {
        val distanceInMeters = flywheelDistanceToTarget.meters
        val calculatedRPMs = FlywheelConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT.get(distanceInMeters)

        return AngularVelocity.fromRpm(calculatedRPMs)
    }

    /**
     * Calculate the Scoring TOF [Flywheel] target [AngularVelocity] based on the distance to its target.
     * And making use of [FlywheelConstants.StoredPolynomials.ToFPolynomial]
     */
    fun getCalculatedScoringTimeOfFlight(flywheelDistanceToTarget: Distance): Time {
        val distanceInMeters = flywheelDistanceToTarget.meters
        val calculatedTOF = FlywheelConstants.StoredPolynomials.ToFPolynomial.getOrThrow().evaluate(distanceInMeters)

        return Time(calculatedTOF)
    }

    fun getRPMDrop(): AngularVelocity {
        return flywheelTargetVelocity.minus(flywheelMotor.getVelocity().get())
    }

    /**
     * @return whether the flywheel has reached its target
     */
    fun getIsAtTarget(): Boolean {
        if (flywheelTargetVelocity < FlywheelConstants.Mechanical.VELOCITY_TARGET_THRESHOLD) return false

        return atTargetDebouncer.calculate(bangBangController.atSetpoint())
    }

    // Logs useful values
    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Flywheel")
        telemetry.addData("Flywheel Lead Motor Connected", flywheelMotor.getIsConnected().asBoolean)
        telemetry.addData("Flywheel Target Velocity RPM", flywheelTargetVelocity.rpm)
        telemetry.addData("Flywheel Velocity RPM", flywheelMotor.getVelocity().get().rpm)
        telemetry.addData("Flywheel Error RPM", bangBangController.getError())
        telemetry.addData("Flywheel BangBang Tolerance RPM", bangBangController.getTolerance())
        telemetry.addData("At target", getIsAtTarget())
    }

    // Motor and interpolation configuration
    private fun configureMotors() {
        flywheelMotor = OpMotorEx(hardwareMap, FlywheelConstants.Identification.FLYWHEEL_LEAD_MOTOR_ID)
        flywheelMotor.applyConfigurationAndResetEncoder(FlywheelConstants.Configuration.leadMotorConfiguration)

        bangBangController.setTolerance(SubsystemTolerances.FLYWHEEL_RPM_TOLERANCE.rpm)

        FlywheelConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT.createLUT()
    }
}