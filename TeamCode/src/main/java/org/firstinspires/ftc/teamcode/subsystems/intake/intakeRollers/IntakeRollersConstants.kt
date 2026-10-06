package org.firstinspires.ftc.teamcode.subsystems.intake.intakeRollers

import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.utils.configurations.OpMotorExConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.genericConfigurations.GenericMotorConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity

object IntakeRollersConstants {
    
    object Identification {
        const val INTAKE_ROLLERS_MOTOR_ID = "rollersMotor"
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
            .withVelocityCoefficients(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_PID)
            .withFeedforwardCoefficients(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_FEEDFORWARD)

        val intakeRollersConfiguration = OpMotorExConfiguration()
            .withGenericMotorConfiguration(motorGenericConfiguration)
            .withControlModeConfiguration(velocityModeConfiguration)
    }
}