package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel

import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.util.InterpLUT
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.utils.configurations.OpMotorExConfiguration
import org.firstinspires.ftc.teamcode.utils.controlProfiles.Polynomials
import org.firstinspires.ftc.teamcode.utils.controllers.invertSigns
import org.firstinspires.ftc.teamcode.utils.devices.configurations.genericConfigurations.GenericMotorConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorPercentageModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.teamcode.utils.units.Time

object FlywheelConstants {

    object Identification {
        const val FLYWHEEL_LEAD_MOTOR_ID = "flywheelLeadMotor"
    }

    object Mechanical {
        const val GEAR_RATIO = (50.0 / 30.0)
        val VELOCITY_TARGET_THRESHOLD = AngularVelocity.fromRpm(500.0)
    }

    object Interpolation {
        val SCORING_HIVE_INTERPOLATED_LUT: InterpLUT = InterpLUT()
            .add(0.75, 1700.0)
            .add(1.00, 1710.0)
            .add(1.25, 1770.0)
            .add(1.50, 1810.0)
    }


    object Control {
        /**
         * Fraction of the feedforward applied alongside the bang-bang output.
         * Slightly under 1.0 (WPILib recommends 0.9) so the feedforward alone never
         * pushes the flywheel past the setpoint, letting bang-bang handle the final approach.
         */
        const val FEEDFORWARD_SCALE = 0.94
    }

    object Configuration {
        private const val INVERTED = true

        private const val MAX_POWER = 1.0

        private val zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT

        private val motorGenericConfiguration = GenericMotorConfiguration()
            .withInverted(INVERTED)
            .withGearRatio(Mechanical.GEAR_RATIO)
            .withZeroPowerBehavior(zeroPowerBehavior)

        private val percentageModeConfiguration = MotorPercentageModeConfiguration()
            .withMaxPower(MAX_POWER)

        val leadMotorConfiguration = OpMotorExConfiguration()
            .withGenericMotorConfiguration(motorGenericConfiguration)
            .withControlModeConfiguration(percentageModeConfiguration)
    }
    object StoredPolynomials {
        val scoringPolynomial : Result<Polynomials> = Polynomials.of(2, -218.942, 590.854, 1290.0)
        val ToFPolynomial : Result<Polynomials> = Polynomials.of(2,0.06903, -0.05319, 0.25993)
    }
}