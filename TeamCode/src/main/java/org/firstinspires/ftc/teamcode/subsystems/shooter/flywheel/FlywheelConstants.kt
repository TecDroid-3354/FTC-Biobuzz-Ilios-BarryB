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

// TODO Before test validate these values
object FlywheelConstants {

    object Identification {
        const val FLYWHEEL_LEAD_MOTOR_ID = "flywheelLeadMotor"
    }

    object Mechanical {
        const val GEAR_RATIO = 1.0
        val VELOCITY_TARGET_THRESHOLD = AngularVelocity.fromRpm(500.0)
    }

    object Interpolation {
        val SCORING_POINTS_LIST: Map<Distance, AngularVelocity> = mapOf(
            Distance(0.0) to AngularVelocity(0.0),
            Distance(0.25) to AngularVelocity(0.1),
            Distance(0.50) to AngularVelocity(0.2),
            Distance(0.75) to AngularVelocity(0.3),
            Distance(1.00) to AngularVelocity(0.4),
            Distance(1.25) to AngularVelocity(0.5),
            Distance(1.50) to AngularVelocity(0.6),
            Distance(1.75) to AngularVelocity(0.7),
            Distance(2.00) to AngularVelocity(0.8),
            Distance(2.25) to AngularVelocity(0.9),
            Distance(2.50) to AngularVelocity(0.95),
            Distance(2.75) to AngularVelocity(0.99)
        )

        val SCORING_HIVE_INTERPOLATED_LUT = InterpLUT()

        val TIME_OF_FLIGHT_POINTS_LIST: Map<Distance, Time> = mapOf(
            Distance(0.0) to Time(0.0),
            Distance(0.25) to Time(0.1),
            Distance(0.50) to Time(0.2),
            Distance(0.75) to Time(0.3),
            Distance(1.00) to Time(0.4),
            Distance(1.25) to Time(0.5),
            Distance(1.50) to Time(0.6),
            Distance(1.75) to Time(0.7),
            Distance(2.00) to Time(0.8),
            Distance(2.25) to Time(0.9),
            Distance(2.50) to Time(0.95),
            Distance(2.75) to Time(0.99)
        )

        val TIME_OF_FLIGHT_HIVE_INTERPOLATED_LUT = InterpLUT()
    }

    object Configuration {
        private const val INVERTED = true

        private val zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT

        private val motorGenericConfiguration = GenericMotorConfiguration()
            .withInverted(INVERTED)
            .withGearRatio(Mechanical.GEAR_RATIO)
            .withZeroPowerBehavior(zeroPowerBehavior)

        private val velocityModeConfiguration = MotorVelocityModeConfiguration()
            .withVelocityCoefficients(SubsystemControlGains.FLYWHEEL_MOTOR_PID)
            .withFeedforwardCoefficients(SubsystemControlGains.FLYWHEEL_MOTOR_FEEDFORWARD)

        val leadMotorConfiguration = OpMotorExConfiguration()
            .withGenericMotorConfiguration(motorGenericConfiguration)
            .withControlModeConfiguration(velocityModeConfiguration)
    }
}