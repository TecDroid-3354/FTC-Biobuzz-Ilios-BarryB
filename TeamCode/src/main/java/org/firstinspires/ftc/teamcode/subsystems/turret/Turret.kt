package org.firstinspires.ftc.teamcode.subsystems.turret

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.util.Debouncer
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants
import org.firstinspires.ftc.teamcode.utils.devices.RTPServo.Direction
import java.util.Formatter
import java.util.function.Supplier
import kotlin.math.abs

class Turret(private val hardwareMap: HardwareMap): SubsystemBase() {

    private lateinit var leftTurretServo    : OpServoEx

    private lateinit var rightTurretServo   : OpServoEx

    private lateinit var turretEncoder      : AnalogInput

    private var turretTargetAngle: Angle = Angle(0.0)

    private var configurableTurretAngle: Angle = Angle(0.0)

    private val atTargetDebouncer: Debouncer = Debouncer(0.06, Debouncer.DebounceType.Both)

    init {
        // Initialization code
        configureServos()

        setTurretZeroAngle().run()
    }

    /** Runs every cycle */
    override fun periodic() {
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
        val normalizedAngle = normalizeDegrees(turretTargetAngle.degrees)

        return Angle.fromDegrees(normalizedAngle)
    }

    /**
     * @return whether the flywheel has reached its target
     */
    fun getIsAtTarget(): Boolean {
        val withinTolerance = abs(
            (turretTargetAngle.minus(getServoAbsoluteAngle())).degrees
        ) < SubsystemTolerances.TURRET_ANGLE_TOLERANCE.degrees

        return atTargetDebouncer.calculate(withinTolerance)
    }

    /**
     * Gets the absolute position of the servo, not considering gear ratios
     */
    private fun getServoAbsoluteAngle(): Angle {
        val currentAngle = Angle.fromDegrees(
            (turretEncoder.voltage / TurretConstants.Configuration.ABSOLUTE_MAX_VOLTAGE.volts) * (if (TurretConstants.Configuration.LEFT_INVERTED) -360 else 360)
        )
        val transformedAngle = Angle.fromDegrees(
            (currentAngle.degrees - TurretConstants.Mechanical.TURRET_ENCODER_ANGLE_OFFSET.degrees)
        )

        return Angle.fromDegrees(normalizeDegrees(transformedAngle.degrees))
    }

    // Log useful values
    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Turret")
        telemetry.addData("Turret Angle Degrees", getServoAbsoluteAngle().degrees * TurretConstants.Mechanical.GEAR_RATIO)
        telemetry.addData("Turret Target Angle Degrees", turretTargetAngle.degrees)
        telemetry.addData("Turret at target", getIsAtTarget())
    }

    // Servo configuration
    fun configureServos() {
        leftTurretServo = OpServoEx(hardwareMap, TurretConstants.Identification.TURRET_LEFT_SERVO_ID)
        leftTurretServo.applyConfiguration(TurretConstants.Configuration.leftTurretServoConfiguration)

        rightTurretServo = OpServoEx(hardwareMap, TurretConstants.Identification.TURRET_RIGHT_SERVO_ID)
        rightTurretServo.applyConfiguration(TurretConstants.Configuration.rightTurretServoConfiguration)

        turretEncoder = hardwareMap.get(AnalogInput::class.java, TurretConstants.Identification.TURRET_ABSOLUTE_ID)
    }
}