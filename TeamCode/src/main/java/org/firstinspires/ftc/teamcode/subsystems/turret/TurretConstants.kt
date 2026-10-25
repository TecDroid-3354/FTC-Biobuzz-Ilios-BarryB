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
        const val TURRET_LEFT_SERVO_ID = "rightTurretServo"

        const val TURRET_RIGHT_SERVO_ID = "leftTurretServo"
    }

    object Mechanical {
        const val GEAR_RATIO = 1.0
        val TARGET_ANGLE_THRESHOLD = Angle.fromDegrees(1.0)

        val turretServosRange = Angle.fromDegrees(0.0)..Angle.fromDegrees(360.0)
    }

    object Configuration {
        private const val LEFT_INVERTED = false

        private const val RIGHT_INVERTED = false

        val leftTurretServoConfiguration = ServoPositionModeConfiguration()
            .withInverted(LEFT_INVERTED)
            .withRange(Mechanical.turretServosRange.endInclusive)

        val rightTurretServoConfiguration = leftTurretServoConfiguration
            .withInverted(RIGHT_INVERTED)
    }
}