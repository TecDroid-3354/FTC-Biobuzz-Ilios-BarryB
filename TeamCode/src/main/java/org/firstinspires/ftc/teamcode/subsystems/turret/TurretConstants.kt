package org.firstinspires.ftc.teamcode.subsystems.turret

import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoRunToPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Voltage

object TurretConstants {

    object Identification {
        const val TURRET_LEFT_SERVO_ID = "rightTurretServo"
        const val LEFT_ABSOLUTE_ENCODER_ID = "turretLeftAbs"

        const val TURRET_RIGHT_SERVO_ID = "leftTurretServo"
        const val RIGHT_ABSOLUTE_ENCODER_ID = "turretRightAbs"
    }

    object Mechanical {
        const val GEAR_RATIO = 1.0
        val TARGET_ANGLE_THRESHOLD = Angle.fromDegrees(1.0)

        val leftEncoderOffset = Angle(0.0)

        val rightEncoderOffset = Angle(0.0)
    }

    object Configuration {
        private const val LEFT_INVERTED = false

        private const val RIGHT_INVERTED = false

        private const val TURRET_SERVOS_MAX_POWER = 0.99

        private val leftAbsoluteMaxVoltage = Voltage(3.3)

        private val rightAbsoluteMaxVoltage = Voltage(3.3)

        val leftTurretServoConfiguration = ServoRunToPositionModeConfiguration()
            .withInverted(LEFT_INVERTED)
            .withGearRatio(Mechanical.GEAR_RATIO)
            .withMaxPower(TURRET_SERVOS_MAX_POWER)
            .withAbsoluteId(Identification.LEFT_ABSOLUTE_ENCODER_ID)
            .withEncoderOffset(Mechanical.leftEncoderOffset)
            .withAbsoluteMaxVoltage(leftAbsoluteMaxVoltage)
            .withPositionLimits(SubsystemLimits.TURRET_ANGLE_LIMITS)
            .withPIDFCoefficients(SubsystemControlGains.TURRET_SERVOS_PIDF)
            .withPositionTolerance(SubsystemTolerances.TURRET_ANGLE_TOLERANCE)

        val rightTurretServoConfiguration = leftTurretServoConfiguration
            .withInverted(RIGHT_INVERTED)
            .withAbsoluteId(Identification.RIGHT_ABSOLUTE_ENCODER_ID)
            .withEncoderOffset(Mechanical.rightEncoderOffset)
            .withAbsoluteMaxVoltage(rightAbsoluteMaxVoltage)
    }
}