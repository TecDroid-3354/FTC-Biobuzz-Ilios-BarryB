package org.firstinspires.ftc.teamcode.subsystems.shooter.hood

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx
import org.firstinspires.ftc.teamcode.utils.extensions.InstantCommand
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.robotcore.external.Supplier
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelConstants
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelConstants.Interpolation
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity
import kotlin.math.abs

class Hood(private val hardwareMap: HardwareMap): SubsystemBase() {

    private lateinit var hoodServo: OpServoEx

    private var hoodTargetAngle: Angle = Angle(0.0)

    init {
        // Initialization code
        configureServo()
    }

    /**
     * A [Runnable] which sets a new [Angle] after being clamped by the [SubsystemLimits.HOOD_ANGLE_LIMITS] limit.
     * @param angle the desired [Angle]
     * @return a [Runnable] which sets a new angle
     */
    private fun setHoodPosition(angle: Angle): Runnable {
        return Runnable {
            val clampedAngle = angle.coerceIn(SubsystemLimits.HOOD_ANGLE_LIMITS)
            val transformedAngle = clampedAngle.degrees * HoodConstants.Mechanical.GEAR_RATIO
            hoodTargetAngle = Angle.fromDegrees(transformedAngle)

            hoodServo.setServoPosition(Angle.fromDegrees(transformedAngle))
        }
    }

    /**
     * Calls [setHoodPosition] and sets a new angle with the [SubsystemPresetTargets.HOOD_PRESET_ANGLE] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets the new hood angle.
     */
    fun setHoodPresetAngle(): Command {
        return setHoodPosition(SubsystemPresetTargets.HOOD_PRESET_ANGLE).InstantCommand(this)
    }

    /**
     * Calls [setHoodPosition] and sets a new angle with the [SubsystemPresetTargets.HOOD_HOME_ANGLE] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets the new hood angle.
     */
    fun setHoodHomeAngle(): Command {
        return setHoodPosition(SubsystemPresetTargets.HOOD_HOME_ANGLE).InstantCommand(this)
    }

    /**
     * Calls [setHoodPosition] and sets a new angle with the [SubsystemConfigurableTargets.HOOD_CONFIGURABLE_DEGREES] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets the new hood angle.
     */
    fun setHoodConfigurableAngle(): Command {
        return setHoodPosition(Angle.fromDegrees(SubsystemConfigurableTargets.HOOD_CONFIGURABLE_DEGREES)).InstantCommand(this)
    }

    /**
     * Calculate the desired target angle based on the [flywheelDistanceToTarget] and the [HoodConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT].
     * @return a [RunCommand] that sets the new calculated angle.
     */
    fun setHoodCalculatedAngle(flywheelDistanceToTarget: Supplier<Distance>): Command {
        return RunCommand({
            setHoodPosition(getCalculatedHoodScoringAngle(flywheelDistanceToTarget.get())).run()
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

    /**
     * @return whether the flywheel has reached its target
     */
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