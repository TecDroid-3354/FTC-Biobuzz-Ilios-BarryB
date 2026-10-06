package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheelHardStop

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx
import org.firstinspires.ftc.teamcode.utils.extensions.InstantCommand
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity
import kotlin.math.abs

@Suppress("JoinDeclarationAndAssignment")
class FlywheelHardStop(hardwareMap: HardwareMap): SubsystemBase() {

    private val flywheelHardStopServo: OpServoEx

    private var flywheelHardStopTargetAngle: Angle = Angle(0.0)

    init {
        // Initialization code
        flywheelHardStopServo = OpServoEx(hardwareMap, FlywheelHardStopConstants.Identification.FLYWHEEL_HARD_STOP_SERVO_ID)
        flywheelHardStopServo.applyConfiguration(FlywheelHardStopConstants.Configuration.servoConfiguration)
    }

    /**
     * A [Runnable] which sets a new target [Angle] after being clamped by the [SubsystemLimits.FLYWHEEL_HARD_STOP_LIMITS] limit.
     * @param angle the desired [Angle]
     * @return a [Runnable] which sets a new angle
     */
    private fun setHardStopAngle(angle: Angle): Runnable {
        return {
            val clampedAngle = angle.coerceIn(SubsystemLimits.FLYWHEEL_HARD_STOP_LIMITS)
            flywheelHardStopTargetAngle = angle

            flywheelHardStopServo.setServoPosition(clampedAngle)
        }
    }

    /**
     * Calls [setHardStopAngle] and sets a new angle with the [SubsystemPresetTargets.FLYWHEEL_HARD_STOP_CLEARING_ANGLE] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets a new angle.
     */
    fun setFlywheelHardStopClearingAngle(): Command {
        return setHardStopAngle(SubsystemPresetTargets.FLYWHEEL_HARD_STOP_CLEARING_ANGLE).InstantCommand(this)
    }

    /**
     * Calls [setHardStopAngle] and sets a new angle with the [SubsystemPresetTargets.FLYWHEEL_HARD_STOP_BLOCKING_ANGLE] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets a new angle.
     */
    fun setFlywheelHardStopBlockingAngle(): Command {
        return setHardStopAngle(SubsystemPresetTargets.FLYWHEEL_HARD_STOP_BLOCKING_ANGLE).InstantCommand(this)
    }

    /**
     * Calls [setHardStopAngle] and sets a new angle with the [SubsystemConfigurableTargets.FLYWHEEL_CONFIGURABLE_RPM] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets a new angle.
     */
    fun setFlywheelHardStopConfigurableAngle(): Command {
        return setHardStopAngle(Angle.fromDegrees(SubsystemConfigurableTargets.FLYWHEEL_HARD_STOP_CONFIGURABLE_ANGLE)).InstantCommand(this)
    }

    /**
     * @return whether the flywheel has reached its target
     */
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