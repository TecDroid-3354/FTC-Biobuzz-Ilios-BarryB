package org.firstinspires.ftc.teamcode.subsystems.shooter.hood

import com.seattlesolvers.solverslib.util.InterpLUT
import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Distance

// TODO Before test validate these values
object HoodConstants {

    object Identification {
        const val HOOD_SERVO_ID = "hoodServo"
    }

    object Mechanical {
        const val GEAR_RATIO = 1.0
        val TARGET_ANGLE_THRESHOLD = Angle.fromDegrees(2.0)
    }

    object Interpolation {
        val SCORING_POINTS_LIST: Map<Distance, Angle> = mapOf(
            Distance(0.0) to Angle(0.0),
            Distance(0.25) to Angle(0.1),
            Distance(0.50) to Angle(0.2),
            Distance(0.75) to Angle(0.3),
            Distance(1.00) to Angle(0.4),
            Distance(1.25) to Angle(0.5),
            Distance(1.50) to Angle(0.6),
            Distance(1.75) to Angle(0.7),
            Distance(2.00) to Angle(0.8),
            Distance(2.25) to Angle(0.9),
            Distance(2.50) to Angle(0.95),
            Distance(2.75) to Angle(0.99)
        )

        val SCORING_HIVE_INTERPOLATED_LUT = InterpLUT()
    }

    object Configuration {
        private const val INVERTED = false

        val range = Angle(0.0)..Angle.fromDegrees(360.0)

        val hoodServoConfiguration = ServoPositionModeConfiguration()
            .withInverted(INVERTED)
            .withRange(range.endInclusive)
    }
}