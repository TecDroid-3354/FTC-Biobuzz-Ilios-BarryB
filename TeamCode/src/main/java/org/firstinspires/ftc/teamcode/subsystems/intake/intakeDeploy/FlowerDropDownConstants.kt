package org.firstinspires.ftc.teamcode.subsystems.intake.intakeDeploy

import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.Angle

// TODO Before test validate these values
object FlowerDropDownConstants {

    object Identification{
        const val DROP_DOWN_LEAD_SERVO_ID = "l"

        const val DROP_DOWN_FOLLOWER_SERVO_ID = "f"
    }

    object Mechanical {
        const val GEAR_RATIO = 1.0
    }

    object Configuration{
        private const val LEAD_SERVO_INVERTED = false
        private const val FOLLOWER_SERVO_INVERTED = true

        val servoRange = Angle(0.0)..Angle(180.0)

        val leadServoConfiguration = ServoPositionModeConfiguration()
            .withInverted(LEAD_SERVO_INVERTED)
            .withRange(servoRange)

        val followerServoConfiguration = ServoPositionModeConfiguration()
            .withInverted(FOLLOWER_SERVO_INVERTED)
            .withRange(servoRange)
    }
}