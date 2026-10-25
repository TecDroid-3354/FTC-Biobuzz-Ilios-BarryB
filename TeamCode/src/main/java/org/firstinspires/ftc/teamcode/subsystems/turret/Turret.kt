package org.firstinspires.ftc.teamcode.subsystems.turret

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.util.Debouncer
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import java.util.function.Supplier
import kotlin.math.abs

class Turret(private val hardwareMap: HardwareMap): SubsystemBase() {

    private lateinit var leftTurretServo: OpServoEx

    private lateinit var rightTurretServo: OpServoEx

    private var turretTargetAngle: Angle = Angle(0.0)

    private var configurableTurretAngle: Angle = Angle(0.0)

    private val atTargetDebouncer: Debouncer = Debouncer(0.06, Debouncer.DebounceType.Both)

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

        if (configurableTurretAngle.degrees.equals(SubsystemConfigurableTargets.TURRET_CONFIGURABLE_DEGREES).not()) {
            updateTurretConfigurableAngle(SubsystemConfigurableTargets.TURRET_CONFIGURABLE_DEGREES)
        }
    }

    /**
     * Updates [configurableTurretAngle] with the value of [SubsystemConfigurableTargets.TURRET_CONFIGURABLE_DEGREES]
     */
    private fun updateTurretConfigurableAngle(newTurretConfigurableAngleDeg: Double) {
        configurableTurretAngle = Angle.fromDegrees(newTurretConfigurableAngleDeg)
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
     * Sets a new angle with the [SubsystemPresetTargets.TURRET_ZERO_ANGLE] value.
     * @return a [Runnable] which sets the new turret angle.
     */
    fun setTurretZeroAngle(): Runnable {
        return {
            turretTargetAngle = SubsystemPresetTargets.TURRET_ZERO_ANGLE

            rightTurretServo.setServoPosition(SubsystemPresetTargets.TURRET_ZERO_ANGLE,
                SubsystemLimits.TURRET_ANGLE_LIMITS, TurretConstants.Mechanical.GEAR_RATIO)
            leftTurretServo.setServoPosition(SubsystemPresetTargets.TURRET_ZERO_ANGLE,
                SubsystemLimits.TURRET_ANGLE_LIMITS, TurretConstants.Mechanical.GEAR_RATIO)
        }
    }

    /**
     * Sets a new angle with the [SubsystemConfigurableTargets.TURRET_CONFIGURABLE_DEGREES] value.
     * @return a [Runnable] which sets the new turret angle.
     */
    fun setTurretConfigurableAngle(): Runnable {
        return {
            val clampedAngleDeg = MathUtils.clamp(configurableTurretAngle.degrees, SubsystemLimits.TURRET_ANGLE_LIMITS.start.degrees,
                SubsystemLimits.TURRET_ANGLE_LIMITS.endInclusive.degrees)
            turretTargetAngle = Angle.fromDegrees(clampedAngleDeg)

            rightTurretServo.setServoPosition(configurableTurretAngle,
                SubsystemLimits.TURRET_ANGLE_LIMITS, TurretConstants.Mechanical.GEAR_RATIO)
            leftTurretServo.setServoPosition(configurableTurretAngle,
                SubsystemLimits.TURRET_ANGLE_LIMITS, TurretConstants.Mechanical.GEAR_RATIO)
        }
    }

    /**
     * Sets the new [turretCalculatedAngle].
     * @return a [Runnable] that sets the new calculated angle.
     */
    private fun setTurretCalculatedAngle(turretCalculatedAngle: Angle): Runnable {
        return {
            val clampedAngleDeg = MathUtils.clamp(turretCalculatedAngle.degrees, SubsystemLimits.TURRET_ANGLE_LIMITS.start.degrees,
                SubsystemLimits.TURRET_ANGLE_LIMITS.endInclusive.degrees)
            turretTargetAngle = Angle.fromDegrees(clampedAngleDeg)

            rightTurretServo.setServoPosition(turretCalculatedAngle,
                SubsystemLimits.TURRET_ANGLE_LIMITS, TurretConstants.Mechanical.GEAR_RATIO)
            leftTurretServo.setServoPosition(turretCalculatedAngle,
                SubsystemLimits.TURRET_ANGLE_LIMITS, TurretConstants.Mechanical.GEAR_RATIO)
        }
    }

    fun setTurretCalculatedAngleCommand(chassisRelativeTargetAngle: Supplier<Rotation2d>, chassisCurrentRotation: Supplier<Rotation2d>): Command {
        return RunCommand({
            val turretCalculatedAngle = calculateTurretAngle(chassisRelativeTargetAngle.get(), chassisCurrentRotation.get())
            setTurretCalculatedAngle(turretCalculatedAngle).run()
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

        val withinTolerance = abs(
            (turretTargetAngle.minus(rightTurretServo.getAngle())).degrees
        ) < SubsystemTolerances.TURRET_ANGLE_TOLERANCE.degrees

        return atTargetDebouncer.calculate(withinTolerance)
    }

    // Log useful values
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