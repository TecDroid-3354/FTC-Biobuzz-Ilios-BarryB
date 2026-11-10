package org.firstinspires.ftc.teamcode.subsystems.indexer

import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.utils.configurations.OpMotorExConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.genericConfigurations.GenericMotorConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.sensors.ColorSensorConfig
import org.firstinspires.ftc.teamcode.utils.sensors.DetectedColor
import org.firstinspires.ftc.teamcode.utils.sensors.HueRange
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.units.Time

// TODO Before test validate these values
object IndexerConstants {

    object Identification {
        const val INDEXER_MOTOR_ID = "indexerMotor"

        const val TOP_COLOR_SENSOR_ID = "tcs"
        const val BOTTOM_LEFT_COLOR_SENSOR_ID = "blcs"
        const val BOTTOM_RIGHT_COLOR_SENSOR_ID = "brcs"
    }

    object Mechanical {
        const val GEAR_RATIO = (54.0 / 18.0)
        val TARGET_VELOCITY_THRESHOLD = AngularVelocity.fromRpm(100.0)
    }

    object ColorSensor {
        val TOP_SENSOR_CONFIG = ColorSensorConfig(
            Identification.TOP_COLOR_SENSOR_ID,
            gain = 2.0f,
            ledEnabled = true,
            minSaturation = 0.350f,
            minValue = 0.020f,
            hueRanges = mapOf(
                DetectedColor.RED to HueRange(0.0f, 20.0f),
                DetectedColor.YELLOW to HueRange(92.49489f, 20.0f),
                DetectedColor.BLUE to HueRange(220.0f, 20.0f)
            )
        )

        val BOTTOM_LEFT_SENSOR_CONFIG = ColorSensorConfig(
            Identification.BOTTOM_LEFT_COLOR_SENSOR_ID,
            gain = 2.0f,
            ledEnabled = true,
            minSaturation = 0.0f,
            minValue = 0.0f,
            hueRanges = mapOf(
                DetectedColor.RED to HueRange(123.73f, 2.0f),
                DetectedColor.YELLOW to HueRange(135.8507f, 12.0f),
                DetectedColor.BLUE to HueRange(176.84209f, 5.0f)
            )
        )

        val BOTTOM_RIGHT_SENSOR_CONFIG = ColorSensorConfig(
            Identification.BOTTOM_RIGHT_COLOR_SENSOR_ID,
            gain = 2.0f,
            ledEnabled = true,
            minSaturation = 0.0f,
            minValue = 0.0f,
            hueRanges = mapOf(
                DetectedColor.RED to HueRange(54.999985f, 12.0f),
                DetectedColor.YELLOW to HueRange(109.11291f, 12.0f),
                DetectedColor.RED to HueRange(193.47691f, 12.0f)
            )
        )

        val REQUIRED_DETECTION_TIME_CONSISTENCY_FOR_INDEXER_FULL = Time(1.0)
    }

    object Configuration {
        private const val INVERTED = true
        private val zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        private val motorGenericConfiguration = GenericMotorConfiguration()
            .withGearRatio(Mechanical.GEAR_RATIO)
            .withInverted(INVERTED)
            .withZeroPowerBehavior(zeroPowerBehavior)

        private val velocityModeConfiguration = MotorVelocityModeConfiguration()
            .withVelocityCoefficients(SubsystemControlGains.INDEXER_MOTOR_PID)
            .withFeedforwardCoefficients(SubsystemControlGains.INDEXER_MOTOR_FEEDFORWARD)

        val indexerConfiguration = OpMotorExConfiguration()
            .withGenericMotorConfiguration(motorGenericConfiguration)
            .withControlModeConfiguration(velocityModeConfiguration)
    }
}