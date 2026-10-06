package org.firstinspires.ftc.teamcode.subsystems.turret

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx
import org.firstinspires.ftc.teamcode.utils.extensions.InstantCommand
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.robotcore.external.Supplier
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.subsystems.shooter.hood.HoodConstants
import org.firstinspires.ftc.teamcode.subsystems.shooter.hood.HoodConstants.Interpolation
import org.firstinspires.ftc.teamcode.subsystems.shooter.hood.HoodConstants.Interpolation.SCORING_HIVE_INTERPOLATED_LUT
import org.threeten.bp.OffsetTime
import java.util.Optional
import kotlin.math.abs

class Turret(private val hardwareMap: HardwareMap): SubsystemBase() {

    private lateinit var leftTurretServo: OpServoEx

    private lateinit var rightTurretServo: OpServoEx

    private var turretTargetAngle: Angle = Angle(0.0)

    init {
        // Initialization code
        configureServos()
    }

    /** Runs every cycle */
    override fun periodic() {
        // Both servos have the same PIDF, just check if one had its coefficients updated.
        if (leftTurretServo.hadRTPCoefficientsUpdated(SubsystemControlGains.TURRET_SERVOS_PIDF)) {
            updateTurretPIDF(SubsystemControlGains.TURRET_SERVOS_PIDF)
        }
    }

    /**
     * Updates the [rightTurretServo] and [leftTurretServo]'s [com.qualcomm.robotcore.hardware.PIDFCoefficients]
     * by calling [OpServoEx.updateRunToPositionPIDF]
     * @param pidfCoefficients the new [PIDCoefficients]
     */
    private fun updateTurretPIDF(pidfCoefficients: PIDFCoefficients) {
        leftTurretServo.updateRunToPositionPIDF(pidfCoefficients)
        rightTurretServo.updateRunToPositionPIDF(pidfCoefficients)
    }

    /**
     * A [Runnable] which sets a new [Angle] after being clamped by the [SubsystemLimits.TURRET_ANGLE_LIMITS] limit.
     * @param angle the desired [Angle]
     * @return a [Runnable] which sets a new angle
     */
    private fun setTurretAngle(angle: Angle): Runnable {
        return {
            val clampedAngle = angle.coerceIn(SubsystemLimits.TURRET_ANGLE_LIMITS)
            turretTargetAngle = clampedAngle

            leftTurretServo.runToPosition(clampedAngle)
            rightTurretServo.runToPosition(clampedAngle)
        }
    }

    /**
     * Calls [setTurretAngle] and sets a new angle with the [SubsystemPresetTargets.TURRET_ZERO_ANGLE] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets the new turret angle.
     */
    fun setTurretZeroAngle(): Command {
        return setTurretAngle(SubsystemPresetTargets.TURRET_ZERO_ANGLE).InstantCommand(this)
    }

    /**
     * Calls [setTurretAngle] and sets a new angle with the [SubsystemConfigurableTargets.TURRET_CONFIGURABLE_DEGREES] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets the new turret angle.
     */
    fun setTurretConfigurableAngle(): Command {
        return setTurretAngle(Angle.fromDegrees(SubsystemConfigurableTargets.TURRET_CONFIGURABLE_DEGREES)).InstantCommand(this)
    }

    /**
     * Calculate the desired target angle based on the [chassisRelativeTargetAngle] and the [chassisCurrentRotation].
     * @return a [RunCommand] that sets the new calculated angle.
     */
    fun setCalculatedTurretAngle(chassisRelativeTargetAngle: Supplier<Rotation2d>, chassisCurrentRotation: Supplier<Rotation2d>): Command {
        return RunCommand({
            val targetAngle = calculateTurretAngle(chassisRelativeTargetAngle.get(), chassisCurrentRotation.get())

            setTurretAngle(targetAngle).run()
        }, this)
    }

    /**
     * Calculates the turret-relative target angle based on the chassis-relative target angle and the robot's heading.
     * @return the turret-relative target angle
     */
    private fun calculateTurretAngle(chassisRelativeTargetAngle: Rotation2d, chassisCurrentRotation: Rotation2d): Angle {
        val turretTargetAngle = chassisRelativeTargetAngle.minus(chassisCurrentRotation)

        return Angle.fromDegrees(turretTargetAngle.degrees)
    }

    /**
     * The turret's MEASURED angle (absolute encoder), 0 = facing the robot's forward, counter-clockwise positive.
     * Localization needs the real angle, not the target, because the camera is physically wherever the turret is.
     */
    fun getAngle(): Angle {
        return rightTurretServo.getAngle()
    }

    /**
     * @return whether the flywheel has reached its target
     */
    fun getIsAtTarget(): Boolean {
        if (turretTargetAngle < TurretConstants.Mechanical.TARGET_ANGLE_THRESHOLD) return false

        return abs(
            (turretTargetAngle.minus(rightTurretServo.getAngle())).degrees
        ) < SubsystemTolerances.TURRET_ANGLE_TOLERANCE.degrees
    }


    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Turret")
        telemetry.addData("Turret Right Angle Degrees", rightTurretServo.getAngle().degrees)
        telemetry.addData("Turret Left Angle Degrees", leftTurretServo.getAngle().degrees)
        telemetry.addData("Turret Target Angle Degrees", turretTargetAngle.degrees)
    }

    // Servo configuration
    fun configureServos() {
        leftTurretServo = OpServoEx(hardwareMap, TurretConstants.Identification.TURRET_LEFT_SERVO_ID)
        leftTurretServo.applyConfiguration(TurretConstants.Configuration.leftTurretServoConfiguration)

        rightTurretServo = OpServoEx(hardwareMap, TurretConstants.Identification.TURRET_RIGHT_SERVO_ID)
        rightTurretServo.applyConfiguration(TurretConstants.Configuration.rightTurretServoConfiguration)
    }
}