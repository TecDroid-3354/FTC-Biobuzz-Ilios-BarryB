package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel

import org.firstinspires.ftc.robotcore.external.Supplier
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.command.InstantCommand
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.devices.OpMotorEx
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.extensions.InstantCommand
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.teamcode.utils.units.Time
import kotlin.collections.iterator
import kotlin.math.abs

class Flywheel(private val hardwareMap: HardwareMap): SubsystemBase() {

    private lateinit var leadFlywheelMotor: OpMotorEx
    private lateinit var followerFlywheelMotor: OpMotorEx

    private var flywheelTargetVelocity: AngularVelocity = AngularVelocity(0.0)

    init {
        configureMotors()
    }

    override fun periodic() {
        if (leadFlywheelMotor.hadVelocityPIDControlGainsUpdated(SubsystemControlGains.FLYWHEEL_MOTOR_PID)
            || leadFlywheelMotor.hadVelocityFeedforwardControlGainsUpdated(SubsystemControlGains.FLYWHEEL_MOTOR_FEEDFORWARD)) {
            updateIntakeRollersControlGains(SubsystemControlGains.FLYWHEEL_MOTOR_PID, SubsystemControlGains.FLYWHEEL_MOTOR_FEEDFORWARD)
        }
    }

    private fun updateIntakeRollersControlGains(pidCoefficients: PIDCoefficients, feedforward: SimpleMotorFeedforward) {
        val newConfig = MotorVelocityModeConfiguration()
            .withVelocityCoefficients(pidCoefficients)
            .withFeedforwardCoefficients(feedforward)

        leadFlywheelMotor.applyModeConfiguration(newConfig)
        followerFlywheelMotor.applyModeConfiguration(newConfig)
    }

    private fun enableFlywheelWithVelocity(velocity: AngularVelocity): Runnable {
        return {
            flywheelTargetVelocity = velocity.coerceIn(SubsystemLimits.SHOOTER_MAX_VELOCITY)

            leadFlywheelMotor.setVelocity(velocity)
            followerFlywheelMotor.setVelocity(velocity)
        }
    }

    fun setFlywheelPresetVelocity(): Command {
        return enableFlywheelWithVelocity(SubsystemPresetTargets.FLYWHEEL_PRESET_RPM).InstantCommand(this)
    }

    fun setFlywheelConfigurableVelocity(): Command {
        return enableFlywheelWithVelocity(AngularVelocity.fromRpm(SubsystemConfigurableTargets.FLYWHEEL_CONFIGURABLE_RPM)).InstantCommand(this)
    }

    fun setFlywheelCalculatedScoringVelocity(flywheelDistanceToTarget: Supplier<Distance>): Command {
        return RunCommand({
            val flywheelCalculatedVelocity =
                getCalculatedHiveScoringVelocity(flywheelDistanceToTarget.get()) // TODO Get this value from an actual interpolated table or polynomial
            enableFlywheelWithVelocity(flywheelCalculatedVelocity).run()
        }, this)
    }

    fun stopFlywheel(): Command {
        return InstantCommand({
            leadFlywheelMotor.stopMotor()
            followerFlywheelMotor.stopMotor()
        }, this)
    }

    private fun getCalculatedHiveScoringVelocity(flywheelDistanceToTarget: Distance): AngularVelocity {
        val distanceInMeters = flywheelDistanceToTarget.meters
        val calculatedRPMs = FlywheelConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT.get(distanceInMeters)

        return AngularVelocity.fromRpm(calculatedRPMs)
    }

    fun getCalculatedScoringTimeOfFlight(flywheelDistanceToTarget: Distance): Time {
        val distanceInMeters = flywheelDistanceToTarget.meters
        val calculatedTOF = FlywheelConstants.Interpolation.TIME_OF_FLIGHT_HIVE_INTERPOLATED_LUT.get(distanceInMeters)

        return Time(calculatedTOF)
    }

    fun getIsAtTarget(): Boolean {
        return abs(
            (flywheelTargetVelocity.minus(leadFlywheelMotor.getVelocity().get())).rpm
        ) < SubsystemTolerances.FLYWHEEL_RPM_TOLERANCE.rpm
    }

    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Flywheel")
        telemetry.addData("Flywheel Lead Motor Connected", leadFlywheelMotor.getIsConnected().asBoolean)
        telemetry.addData("Flywheel Follower Motor Connected", followerFlywheelMotor.getIsConnected().asBoolean)
        telemetry.addData("Flywheel Target Velocity RPM", flywheelTargetVelocity.rpm)
        telemetry.addData("Flywheel Velocity RPM", leadFlywheelMotor.getVelocity().get().rpm)
    }

    private fun configureMotors() {
        leadFlywheelMotor = OpMotorEx(hardwareMap, FlywheelConstants.Identification.FLYWHEEL_LEAD_MOTOR_ID)
        leadFlywheelMotor.applyConfigurationAndResetEncoder(FlywheelConstants.Configuration.leadMotorConfiguration)

        followerFlywheelMotor = OpMotorEx(hardwareMap, FlywheelConstants.Identification.FLYWHEEL_FOLLOWER_MOTOR_ID)
        followerFlywheelMotor.applyConfigurationAndResetEncoder(FlywheelConstants.Configuration.followerMotorConfiguration)

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