package org.firstinspires.ftc.teamcode.subsystems.turret

import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoRunToPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Voltage

// TODO Before test validate these values
object TurretConstants {

    object Identification {
        const val TURRET_LEFT_SERVO_ID = "leftTurretServo"

        const val TURRET_RIGHT_SERVO_ID = "rightTurretServo"

        const val TURRET_ABSOLUTE_ID    = "turretAbs"
    }

    object Mechanical {
        const val GEAR_RATIO = 1.0

        val TURRET_ENCODER_ANGLE_OFFSET = Angle.fromDegrees(0.0)

        val turretServosRange = Angle.fromDegrees(-158.18)..Angle.fromDegrees(156.54)
    }

    object Configuration {
        const val LEFT_INVERTED = false
        private const val RIGHT_INVERTED = false

        val ABSOLUTE_MAX_VOLTAGE = Voltage(3.3)

        val leftTurretServoConfiguration = ServoPositionModeConfiguration()
            .withInverted(LEFT_INVERTED)
            .withRange(Mechanical.turretServosRange)

        val rightTurretServoConfiguration = leftTurretServoConfiguration
            .withInverted(RIGHT_INVERTED)
    }
}