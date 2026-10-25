package org.firstinspires.ftc.teamcode.subsystems.shooter.hood

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import java.util.function.Supplier
import kotlin.math.abs

class Hood(private val hardwareMap: HardwareMap): SubsystemBase() {

    private lateinit var hoodServo: OpServoEx

    private var hoodTargetAngle: Angle = Angle(0.0)

    private var configurableHoodTargetAngle = Angle(0.0)

    init {
        // Initialization code
        configureServo()
    }

    /** Runs every cycle */
    override fun periodic() {
        if (configurableHoodTargetAngle.degrees.equals(SubsystemConfigurableTargets.HOOD_CONFIGURABLE_DEGREES).not()) {
            updateConfigurableHoodAngle(SubsystemConfigurableTargets.HOOD_CONFIGURABLE_DEGREES)
        }
    }

    /**
     * Updates [configurableHoodTargetAngle] with the value of [SubsystemConfigurableTargets.HOOD_CONFIGURABLE_DEGREES]
     */
    private fun updateConfigurableHoodAngle(newHoodAngle: Double) {
        configurableHoodTargetAngle = Angle.fromDegrees(newHoodAngle)
    }

    /**
     * Sets a new angle with the [SubsystemPresetTargets.HOOD_PRESET_ANGLE] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets the new hood angle.
     */
    fun setHoodPresetAngle(): Runnable {
        return {
            hoodTargetAngle = SubsystemPresetTargets.HOOD_PRESET_ANGLE

            hoodServo.setServoPosition(
                SubsystemPresetTargets.HOOD_PRESET_ANGLE,
                SubsystemLimits.HOOD_ANGLE_LIMITS,
                HoodConstants.Mechanical.GEAR_RATIO
            )
        }
    }

    /**
     * Sets a new angle with the [SubsystemPresetTargets.HOOD_HOME_ANGLE] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets the new hood angle.
     */
    fun setHoodHomeAngle(): Runnable {
        return {
            hoodTargetAngle = SubsystemPresetTargets.HOOD_HOME_ANGLE

            hoodServo.setServoPosition(
                SubsystemPresetTargets.HOOD_HOME_ANGLE,
                SubsystemLimits.HOOD_ANGLE_LIMITS,
                HoodConstants.Mechanical.GEAR_RATIO
            )
        }
    }

    /**
     * Sets a new angle with the [SubsystemConfigurableTargets.HOOD_CONFIGURABLE_DEGREES] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets the new hood angle.
     */
    fun setHoodConfigurableAngle(): Runnable {
        return {
            hoodTargetAngle = configurableHoodTargetAngle

            hoodServo.setServoPosition(
                configurableHoodTargetAngle,
                SubsystemLimits.HOOD_ANGLE_LIMITS,
                HoodConstants.Mechanical.GEAR_RATIO
            )
        }
    }

    /**
     * Calculate the desired target angle based on the [hoodCalculatedAngle] and the [HoodConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT].
     * @return a [RunCommand] that sets the new calculated angle.
     */
    private fun setHoodCalculatedAngle(hoodCalculatedAngle: Angle): Runnable {
        return {
            val clampedAngleDeg = MathUtils.clamp(hoodCalculatedAngle.degrees, SubsystemLimits.HOOD_ANGLE_LIMITS.start.degrees,
                SubsystemLimits.HOOD_ANGLE_LIMITS.endInclusive.degrees)

            hoodTargetAngle = Angle.fromDegrees(clampedAngleDeg)

            hoodServo.setServoPosition(
                hoodCalculatedAngle,
                SubsystemLimits.HOOD_ANGLE_LIMITS,
                HoodConstants.Mechanical.GEAR_RATIO
            )
        }
    }

    fun setHoodCalculatedAngle(flywheelDistanceToTarget: Supplier<Distance>): Command {
        return RunCommand({
            val hoodCalculatedAngle = getCalculatedHoodScoringAngle(flywheelDistanceToTarget.get())
            setHoodCalculatedAngle(hoodCalculatedAngle).run()
        }, this)
    }

    /**
     * Calculate the HIVE Scoring [Hood] target [Angle] based on the distance to its target.
     */
    private fun getCalculatedHoodScoringAngle(flywheelDistanceToTarget: Distance): Angle {
        val distanceInMeters = flywheelDistanceToTarget.meters
        val calculatedAngle = HoodConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT.get(distanceInMeters)

        return Angle.fromDegrees(calculatedAngle)
    }

    /** @return whether the flywheel has reached its target */
    fun getIsAtTarget(): Boolean {
        if (hoodTargetAngle < HoodConstants.Mechanical.TARGET_ANGLE_THRESHOLD) return false

        return abs(
            hoodTargetAngle.degrees.minus(hoodServo.getRawPosition() * HoodConstants.Configuration.range.endInclusive.degrees)
        ) < SubsystemTolerances.HOOD_ANGLE_TOLERANCE.degrees
    }

    // Logs useful values
    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Hood")
        telemetry.addData("Hood Position Degrees", hoodServo.getRawPosition() * HoodConstants.Configuration.range.endInclusive.degrees)
        telemetry.addData("Hood Target Position Degrees", hoodTargetAngle.degrees)
    }

    // Servo and interpolation configuration
    fun configureServo() {
        hoodServo = OpServoEx(hardwareMap, HoodConstants.Identification.HOOD_SERVO_ID)
        hoodServo.applyConfiguration(HoodConstants.Configuration.hoodServoConfiguration)

        for (point in HoodConstants.Interpolation.SCORING_POINTS_LIST) {
            HoodConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT.add(point.key.meters, point.value.degrees)
        }

        HoodConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT.createLUT()
    }
}