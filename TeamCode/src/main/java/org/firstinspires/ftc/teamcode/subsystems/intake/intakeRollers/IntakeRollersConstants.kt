package org.firstinspires.ftc.teamcode.subsystems.intake.intakeRollers

import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.utils.configurations.OpMotorExConfiguration
import org.firstinspires.ftc.teamcode.utils.controllers.invertSigns
import org.firstinspires.ftc.teamcode.utils.devices.configurations.genericConfigurations.GenericMotorConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity

// TODO Before test validate these values
object IntakeRollersConstants {
    
    object Identification {
        const val LEAD_INTAKE_ROLLERS_MOTOR_ID = "leadRollersMotor"
        const val FOLLOWER_INTAKE_ROLLERS_MOTOR_ID = "followerRollersMotor"
    }
    
    object Mechanical {
        const val GEAR_RATIO = 1.0
        val TARGET_VELOCITY_THRESHOLD = AngularVelocity.fromRpm(100.0)
    }
    
    object Configuration{
        private const val LEAD_INVERTED = false
        private const val FOLLOWER_INVERTED = false
        private val zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        private val leadMotorGenericConfiguration = GenericMotorConfiguration()
            .withGearRatio(Mechanical.GEAR_RATIO)
            .withInverted(LEAD_INVERTED)
            .withZeroPowerBehavior(zeroPowerBehavior)

        private val followerMotorGenericConfiguration = leadMotorGenericConfiguration
            .withInverted(FOLLOWER_INVERTED)

        private val leadVelocityModeConfiguration = MotorVelocityModeConfiguration()
            .withVelocityCoefficients(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_PID)
            .withFeedforwardCoefficients(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_FEEDFORWARD)

        private val followerVelocityModeConfiguration = MotorVelocityModeConfiguration()
            .withVelocityCoefficients(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_PID.invertSigns())
            .withFeedforwardCoefficients(SubsystemControlGains.INTAKE_ROLLERS_MOTOR_FEEDFORWARD.invertSigns())

        val leadMotorConfiguration = OpMotorExConfiguration()
            .withGenericMotorConfiguration(leadMotorGenericConfiguration)
            .withControlModeConfiguration(leadVelocityModeConfiguration)

        val followerMotorConfiguration = OpMotorExConfiguration()
            .withGenericMotorConfiguration(followerMotorGenericConfiguration)
            .withControlModeConfiguration(followerVelocityModeConfiguration)
    }
}