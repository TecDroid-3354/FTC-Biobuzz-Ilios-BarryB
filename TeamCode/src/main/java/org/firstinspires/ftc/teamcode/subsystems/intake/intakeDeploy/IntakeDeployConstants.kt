package org.firstinspires.ftc.teamcode.subsystems.intake.intakeDeploy

import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoRunToPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Voltage

object IntakeDeployConstants {

    object Identification{
        const val INTAKE_DEPLOY_LEAD_SERVO_ID = "deployLeadServo"
        const val INTAKE_DEPLOY_LEAD_SERVO_ABSOLUTE_ID = "deployLeadAbs"

        const val INTAKE_DEPLOY_FOLLOWER_SERVO_ID = "deployFollowerServo"
        const val INTAKE_DEPLOY_FOLLOWER_SERVO_ABSOLUTE_ID = "deployFollowerAbs"
    }

    object Mechanical {
        const val GEAR_RATIO = 1.0
        val TARGET_ANGLE_THRESHOLD = Angle.fromDegrees(20.0)

        val leadAbsoluteEncoderOffset = Angle(0.0)
        val followerAbsoluteEncoderOffset = Angle(0.0)
    }

    object Configuration{
        private const val LEAD_SERVO_INVERTED = false
        private const val FOLLOWER_SERVO_INVERTED = false

        private const val INTAKE_DEPLOY_SERVOS_MAX_POWER = 0.99


        private val leadAbsoluteMaxVoltage = Voltage(3.3)

        private val followerAbsoluteMaxVoltage = Voltage(3.3)

        val leadServoConfiguration = ServoRunToPositionModeConfiguration()
            .withMaxPower(INTAKE_DEPLOY_SERVOS_MAX_POWER)
            .withInverted(LEAD_SERVO_INVERTED)
            .withGearRatio(Mechanical.GEAR_RATIO)
            .withAbsoluteId(Identification.INTAKE_DEPLOY_LEAD_SERVO_ABSOLUTE_ID)
            .withEncoderOffset(Mechanical.leadAbsoluteEncoderOffset)
            .withAbsoluteMaxVoltage(leadAbsoluteMaxVoltage)
            .withPositionLimits(SubsystemLimits.INTAKE_DEPLOY_ANGLE_LIMITS)
            .withPositionTolerance(SubsystemTolerances.INTAKE_DEPLOY_ANGLE_TOLERANCE)
            .withPIDFCoefficients(SubsystemControlGains.INTAKE_DEPLOY_SERVOS_PIDF)

        val followerServoConfiguration = leadServoConfiguration
            .withInverted(FOLLOWER_SERVO_INVERTED)
            .withAbsoluteId(Identification.INTAKE_DEPLOY_FOLLOWER_SERVO_ABSOLUTE_ID)
            .withEncoderOffset(Mechanical.followerAbsoluteEncoderOffset)
            .withAbsoluteMaxVoltage(followerAbsoluteMaxVoltage)
    }
}