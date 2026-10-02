package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheelHardStop

import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.Angle

object FlywheelHardStopConstants {

    object Identification {
        const val FLYWHEEL_HARD_STOP_SERVO_ID = "flywheelStop"
    }

    object Mechanical {
        const val GEAR_RATIO = 1.0

        val servoRange = Angle(0.0)..Angle.fromDegrees(180.0)
    }

    object Configuration {
        private const val INVERTED = false

        val servoConfiguration = ServoPositionModeConfiguration()
            .withInverted(INVERTED)
            .withRange(Mechanical.servoRange.endInclusive)
    }
}