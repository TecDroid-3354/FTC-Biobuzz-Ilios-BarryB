package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheelHardStop

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx
import org.firstinspires.ftc.teamcode.utils.extensions.InstantCommand
import org.firstinspires.ftc.teamcode.utils.units.Angle
import kotlin.math.abs

@Suppress("JoinDeclarationAndAssignment")
class FlywheelHardStop(hardwareMap: HardwareMap): SubsystemBase() {

    private val flywheelHardStopServo: OpServoEx

    private var flywheelHardStopTargetAngle: Angle = Angle(0.0)

    private var configurableFlywheelHardStopAngle: Angle = Angle(0.0)

    init {
        // Initialization code
        flywheelHardStopServo = OpServoEx(hardwareMap, FlywheelHardStopConstants.Identification.FLYWHEEL_HARD_STOP_SERVO_ID)
        flywheelHardStopServo.applyConfiguration(FlywheelHardStopConstants.Configuration.servoConfiguration)
    }

    override fun periodic() {
        if (configurableFlywheelHardStopAngle.degrees.equals(SubsystemConfigurableTargets.FLYWHEEL_HARD_STOP_CONFIGURABLE_ANGLE).not()) {
            updateFlywheelHardStopConfigurableAngle(SubsystemConfigurableTargets.FLYWHEEL_HARD_STOP_CONFIGURABLE_ANGLE)
        }
    }

    private fun updateFlywheelHardStopConfigurableAngle(newFlywheelHardStopConfigurableAngleDegrees: Double) {
        configurableFlywheelHardStopAngle = Angle.fromDegrees(newFlywheelHardStopConfigurableAngleDegrees)
    }

    /**
     * Sets a new angle with the [SubsystemPresetTargets.FLYWHEEL_HARD_STOP_CLEARING_ANGLE] value.
     * @return a [Runnable] which sets a new angle.
     */
    fun setFlywheelHardStopClearingAngle(): Runnable {
        return {
            flywheelHardStopTargetAngle = SubsystemPresetTargets.FLYWHEEL_HARD_STOP_CLEARING_ANGLE

            flywheelHardStopServo.setServoPosition(
                SubsystemPresetTargets.FLYWHEEL_HARD_STOP_CLEARING_ANGLE,
                SubsystemLimits.FLYWHEEL_HARD_STOP_LIMITS,
                FlywheelHardStopConstants.Mechanical.GEAR_RATIO
            )
        }
    }

    /**
     * Sets a new angle with the [SubsystemPresetTargets.FLYWHEEL_HARD_STOP_BLOCKING_ANGLE] value.
     * @return a [Runnable] which sets a new angle.
     */
    fun setFlywheelHardStopBlockingAngle(): Runnable {
        return {
            flywheelHardStopTargetAngle = SubsystemPresetTargets.FLYWHEEL_HARD_STOP_BLOCKING_ANGLE

            flywheelHardStopServo.setServoPosition(
                SubsystemPresetTargets.FLYWHEEL_HARD_STOP_BLOCKING_ANGLE,
                SubsystemLimits.FLYWHEEL_HARD_STOP_LIMITS,
                FlywheelHardStopConstants.Mechanical.GEAR_RATIO
            )
        }
    }

    /**
     * Sets a new angle with the [SubsystemConfigurableTargets.FLYWHEEL_CONFIGURABLE_RPM] value.
     * @return  [Runnable] which sets a new angle.
     */
    fun setFlywheelHardStopConfigurableAngle(): Runnable {
        return {
            val clampedAngleDegrees = MathUtils.clamp(configurableFlywheelHardStopAngle.degrees,
                SubsystemLimits.FLYWHEEL_HARD_STOP_LIMITS.start.degrees, SubsystemLimits.FLYWHEEL_HARD_STOP_LIMITS.endInclusive.degrees)
            flywheelHardStopTargetAngle = Angle.fromDegrees(clampedAngleDegrees)

            flywheelHardStopServo.setServoPosition(
                configurableFlywheelHardStopAngle,
                SubsystemLimits.FLYWHEEL_HARD_STOP_LIMITS,
                FlywheelHardStopConstants.Mechanical.GEAR_RATIO
            )
        }
    }

    /** @return whether the flywheel has reached its target */
    fun getIsAtTarget(): Boolean {
        if (flywheelHardStopTargetAngle < FlywheelHardStopConstants.Mechanical.TARGET_ANGLE_THRESHOLD) return false

        return abs(
            flywheelHardStopTargetAngle.degrees.minus(flywheelHardStopServo.getRawPosition() * FlywheelHardStopConstants.Mechanical.servoRange.endInclusive.degrees)
        ) < SubsystemTolerances.FLYWHEEL_HARD_STOP_ANGLE_TOLERANCE.degrees
    }

    // Logs useful values
    fun log(telemetry: TelemetryManager){
        telemetry.addLine("Flywheel Hard Stop")
        telemetry.addData("Flywheel Hard Stop Degrees", flywheelHardStopServo.getRawPosition() * FlywheelHardStopConstants.Mechanical.servoRange.endInclusive.degrees)
        telemetry.addData("Flywheel Hard Stop Target Angle Degrees", flywheelHardStopTargetAngle.degrees)
    }
}