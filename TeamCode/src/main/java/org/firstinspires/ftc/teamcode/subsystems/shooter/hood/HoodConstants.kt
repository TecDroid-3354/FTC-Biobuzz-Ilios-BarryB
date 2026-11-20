package org.firstinspires.ftc.teamcode.subsystems.shooter.hood

import com.seattlesolvers.solverslib.util.InterpLUT
import org.firstinspires.ftc.teamcode.utils.controlProfiles.Polynomials
import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Distance

// TODO Before test validate these values
object HoodConstants {

    object Identification {
        const val HOOD_SERVO_ID = "hoodServo"
    }

    object Control {
        val DEGREES_PER_RPM_LOST = Angle.fromDegrees(0.0031)
    }

    object Mechanical {
        const val GEAR_RATIO = (26.0 / 85.0) * (182.0 / 20.0)
        val TARGET_ANGLE_THRESHOLD = Angle.fromDegrees(2.0)
    }

    object Interpolation {
        val SCORING_HIVE_INTERPOLATED_LUT: InterpLUT = InterpLUT()
            .add(0.75, 15.0)
            .add(1.00, 16.0)
            .add(1.25, 17.0 - 1.0)
            .add(1.50, 23.0 - 1.0)
    }

    object Configuration {
        private const val INVERTED = false

        val range = Angle(0.0)..Angle.fromDegrees(25.0)

        val hoodServoConfiguration = ServoPositionModeConfiguration()
            .withInverted(INVERTED)
            .withRange(range)
    }

    object StoredPolynomials{
        val hoodAnglePolynomial : Result <Polynomials> = Polynomials.of(2 ,6.7727, -0.0738, -1.8254 + 9.5)
    }
}