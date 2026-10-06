package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel

import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.util.InterpLUT
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.utils.configurations.OpMotorExConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.genericConfigurations.GenericMotorConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.teamcode.utils.units.Time

object FlywheelConstants {

    object Identification {
        const val FLYWHEEL_LEAD_MOTOR_ID = "flywheelLeadMotor"
        const val FLYWHEEL_FOLLOWER_MOTOR_ID = "flywheelFollowerMotor"
    }

    object Mechanical {
        const val GEAR_RATIO = 1.0
        val VELOCITY_TARGET_THRESHOLD = AngularVelocity.fromRpm(1000.0)
    }

    object Interpolation {
        val SCORING_POINTS_LIST: Map<Distance, AngularVelocity> = mapOf(
            Distance(0.0) to AngularVelocity(0.0),
            Distance(0.25) to AngularVelocity(0.0),
            Distance(0.50) to AngularVelocity(0.0),
            Distance(0.75) to AngularVelocity(0.0),
            Distance(1.00) to AngularVelocity(0.0),
            Distance(1.25) to AngularVelocity(0.0),
            Distance(1.50) to AngularVelocity(0.0),
            Distance(1.75) to AngularVelocity(0.0),
            Distance(2.00) to AngularVelocity(0.0),
            Distance(2.25) to AngularVelocity(0.0),
            Distance(2.50) to AngularVelocity(0.0),
            Distance(2.75) to AngularVelocity(0.0)
        )

        val SCORING_HIVE_INTERPOLATED_LUT = InterpLUT()

        val TIME_OF_FLIGHT_POINTS_LIST: Map<Distance, Time> = mapOf(
            Distance(0.0) to Time(0.0),
            Distance(0.25) to Time(0.0),
            Distance(0.50) to Time(0.0),
            Distance(0.75) to Time(0.0),
            Distance(1.00) to Time(0.0),
            Distance(1.25) to Time(0.0),
            Distance(1.50) to Time(0.0),
            Distance(1.75) to Time(0.0),
            Distance(2.00) to Time(0.0),
            Distance(2.25) to Time(0.0),
            Distance(2.50) to Time(0.0),
            Distance(2.75) to Time(0.0)
        )

        val TIME_OF_FLIGHT_HIVE_INTERPOLATED_LUT = InterpLUT()
    }

    object Configuration {
        private const val LEAD_INVERTED = false
        private const val FOLLOWER_INVERTED = false

        private val zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT

        private val leadMotorGenericConfiguration = GenericMotorConfiguration()
            .withInverted(LEAD_INVERTED)
            .withGearRatio(Mechanical.GEAR_RATIO)
            .withZeroPowerBehavior(zeroPowerBehavior)

        private val followerMotorGenericConfiguration = leadMotorGenericConfiguration
            .withInverted(FOLLOWER_INVERTED)

        private val velocityModeConfiguration = MotorVelocityModeConfiguration()
            .withVelocityCoefficients(SubsystemControlGains.FLYWHEEL_MOTOR_PID)
            .withFeedforwardCoefficients(SubsystemControlGains.FLYWHEEL_MOTOR_FEEDFORWARD)

        val leadMotorConfiguration = OpMotorExConfiguration()
            .withGenericMotorConfiguration(leadMotorGenericConfiguration)
            .withControlModeConfiguration(velocityModeConfiguration)

        val followerMotorConfiguration = OpMotorExConfiguration()
            .withGenericMotorConfiguration(followerMotorGenericConfiguration)
            .withControlModeConfiguration(velocityModeConfiguration)
    }
}