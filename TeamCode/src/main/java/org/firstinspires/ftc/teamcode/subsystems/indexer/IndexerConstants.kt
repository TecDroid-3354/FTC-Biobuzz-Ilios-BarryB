package org.firstinspires.ftc.teamcode.subsystems.indexer

import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.utils.configurations.OpMotorExConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.genericConfigurations.GenericMotorConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity

object IndexerConstants {

    object Identification {
        const val INDEXER_MOTOR_ID = "indexerMotor"
    }

    object Mechanical {
        const val GEAR_RATIO = 1.0
        val TARGET_VELOCITY_THRESHOLD = AngularVelocity.fromRpm(100.0)
    }

    object Configuration{
        private const val INVERTED = false
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