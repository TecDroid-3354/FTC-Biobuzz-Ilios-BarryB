package org.firstinspires.ftc.teamcode.utils.devices

import com.bylazar.telemetry.TelemetryManager
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.hardware.motors.CRServo
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import com.seattlesolvers.solverslib.util.InterpLUT
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Voltage
import org.firstinspires.ftc.teamcode.utils.TecDroidRobot
import org.firstinspires.ftc.teamcode.utils.controllers.OpPIDFController
import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoContinuousRotationModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoControlModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration.ServoRunToPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.controlModes.ServoControlMode
import kotlin.math.max
import kotlin.math.min

class OpServoEx(private val hardwareMap: HardwareMap, private val servoId: String) {

    // ------- Control Mode ------ //
    private var controlMode                     : ServoControlMode = ServoControlMode.UNKNOWN
    private var timesConfigured                 : Int              = 0
    // ------- STANDARD Servo Position Mode useful variables ------ //
    private lateinit var servo                  : ServoEx
    private var range                           : Angle = Angle(0.0)
    private val angleLUT                        : InterpLUT = InterpLUT()

    // ------- CONTINUOUS ROTATION Servo Mode useful variables ------ //
    private lateinit var crServo                : CRServoEx
    // Max Continuous Rotation Power
    private var maxCRPower                      : Double    = 1.0

    // ------- RUN TO POSITION Servo Mode useful variables ------ //
    private lateinit var rtpServo                : RTPServo

    init {
        registry.add(this)
    }

    // TODO() Add comment
    fun setServoPosition(angle: Angle, subsystemLimits: ClosedRange<Angle>, reduction: Double) {
        if (controlMode != ServoControlMode.POSITION) {
            return
        }

        val subsystemClampedLimits = MathUtils.clamp(angle.degrees, subsystemLimits.start.degrees, subsystemLimits.endInclusive.degrees)
        val servoTransformedAngle = subsystemClampedLimits * reduction
        val servoClampedRange = servoTransformedAngle.coerceIn(0.0, range.degrees)

        val servoPosition = angleLUT.get(servoClampedRange)

        servo.set(servoPosition)
    }

    /**
     * Sets an output considering the maximum power set in the servo's configuration.
     * @param output the desired output from - 1 to 1
     */
    fun setContinuousRotationOutput(output: Double) {
        if (controlMode != ServoControlMode.CONTINUOUS_ROTATION) {
            return
        }

        val power = max(-maxCRPower, min(maxCRPower, output))
        crServo.set(power)
    }

    // TODO() Add comment
    fun stop() {
        if (controlMode != ServoControlMode.RUN_TO_POSITION || controlMode != ServoControlMode.CONTINUOUS_ROTATION) {
            return
        }

        crServo.set(0.0)
    }

    // TODO() Add comment
    fun runToPosition(angle: Angle) {
        if (controlMode != ServoControlMode.RUN_TO_POSITION) {
            return
        }

        rtpServo.setTargetAngle(angle)
    }

    // TODO() Add comment
    fun updateRunToPositionPIDF(coefficients: PIDFCoefficients) {
        if (controlMode != ServoControlMode.RUN_TO_POSITION) {
            return
        }

        rtpServo.setPIDF(coefficients)
    }

    fun getRawPosition(): Double {
        if (controlMode != ServoControlMode.POSITION) {
            return 0.0
        }

        return servo.rawPosition
    }

    // TODO() Add comment
    fun getAngle(): Angle {
        if (controlMode != ServoControlMode.RUN_TO_POSITION) {
            return Angle(0.0)
        }

        return rtpServo.getAngle()
    }

    fun getRTPCoefficients(): PIDFCoefficients {
        if (controlMode != ServoControlMode.RUN_TO_POSITION) {
            return PIDFCoefficients()
        }

        return rtpServo.getPIDFCoefficients()
    }

    fun hadRTPCoefficientsUpdated(pidfCoefficients: PIDFCoefficients): Boolean {
        if (controlMode != ServoControlMode.RUN_TO_POSITION) {
            return false
        }

        return rtpServo.hadPIDFCoefficientsUpdated(pidfCoefficients)
    }

    fun logRTP(telemetry: TelemetryManager) {
        if (controlMode != ServoControlMode.RUN_TO_POSITION) {
            return
        }

        rtpServo.log(telemetry)
    }

    // TODO() Add comment
    private fun updateServo() {
        if (controlMode == ServoControlMode.RUN_TO_POSITION) {
            rtpServo.periodic()
        }
    }

    // TODO() Add comment
    fun applyConfiguration(config: ServoControlModeConfiguration) {
        if (timesConfigured == 0) {
            controlMode = config.controlMode
            timesConfigured++
        } else { return }

        when (config) {
            is ServoPositionModeConfiguration -> {
                servo = ServoEx(hardwareMap, servoId)
                servo.inverted = config.inverted
                range = config.range
                angleLUT.add(0.0, 0.0)
                angleLUT.add(config.range.degrees, 1.0)
                angleLUT.createLUT()
                servo.set(0.0)
            }
            is ServoContinuousRotationModeConfiguration -> {
                crServo = CRServoEx(hardwareMap, servoId)
                crServo.setRunMode(CRServoEx.RunMode.RawPower)
                crServo.inverted = config.inverted
                maxCRPower = config.maxPower
                crServo.set(0.001)
            }
            is ServoRunToPositionModeConfiguration -> {
                rtpServo = RTPServo(
                    hardwareMap,
                    RTPServoConfig(
                        servoId,
                        config.absoluteId,
                        config.absoluteMaxVoltage,
                        if (config.inverted) RTPServo.Direction.REVERSE else RTPServo.Direction.FORWARD,
                        config.encoderOffset,
                        config.maxPower,
                        config.gearRatio,
                        config.pidfCoefficients
                    )
                )
            }
        }
    }

    /**
     * Companion objects belong to the class at top-level, meaning there's just one companion
     * per class creation, not class's instances.
     */
    companion object {
        private val registry = mutableListOf<OpServoEx>()

        /**
         * Calls [OpServoEx.updateServo] on every [OpServoEx] instance that currently exists.
         * Called from [TecDroidRobot] run method to ensure its call no matter what.
         */
        fun updateAll() {
            registry.forEach { it.updateServo() }
        }

        /**
         * Clears the registry of all tracked instances. MUST be called once in the
         * [TecDroidRobot]'s init block, before any [OpServoEx] (or subsystem that creates one)
         * is constructed.
         * Prevents that motors created on previous OpMode are updated in a new one.
         */
        fun clearRegistry() {
            registry.clear()
        }
    }
}

private data class RTPServoConfig(
    val servoId: String,
    val absoluteId: String = "",
    val absoluteMaxVoltage: Voltage,
    val direction: RTPServo.Direction,
    val encoderOffset: Angle,
    val maxPower: Double = 1.0,
    val gearRatio: Double = 1.0,
    val pidfCoefficients: PIDFCoefficients = PIDFCoefficients(0.00575, 0.0, 0.00025, 0.0)
)

@Suppress("JoinDeclarationAndAssignment")
private class RTPServo(hw: HardwareMap, val config: RTPServoConfig) {

    /**
     *[Direction] used for setting a servo direction
     */
    enum class Direction {
        FORWARD, REVERSE;
    }

    // Creating the servo
    private var servo: CRServo

    // Creating the absolute encoder
    private var servoEncoder: AnalogInput

    // Keeps track of the total servo rotation without considering gear ratio
    private var totalRotation: Angle = Angle.fromDegrees(0.0)
    // Keeps track of a previous angle in order to perform a subtraction in each iteration
    private var previousAngle: Angle = Angle.fromDegrees(0.0)
    // Used for setting a target rotation
    private var targetRotation: Angle = Angle.fromDegrees(0.0)
    // Keeps track of the number of full rotations in order to obtain the correct servo position along time
    private var fullRotations = 0

    // The turret controller
    private val pidfController = OpPIDFController(config.pidfCoefficients)

    init {
        /* INITIALIZATION CODE */

        // Initialize both the CR servo and absolute encoder
        servo = hw.get(CRServo::class.java, config.servoId)
        this.servoEncoder = hw.get(AnalogInput::class.java, config.absoluteId)

        // Getting the previous angle reading
        previousAngle = getServoAbsoluteAngle()

        // Setting a default position tolerance
        pidfController.enableContinuousInput(-180.0, 180.0)
        pidfController.setTolerance(1.0)

        // Must call servo.setPower() for correct servo working
        setPower(0.0)
    }

    /**
     * Sets an output considering the maximum power set in the servo's configuration
     * @param output the desired output from - 1 to 1
     */
    fun setPower(output: Double) {
        val power = max(-config.maxPower, min(config.maxPower, output))
        servo.set(power * (if (config.direction == Direction.REVERSE) -1 else 1))
    }

    /**
     * Completely stops the servo's movement
     */
    fun stop() {
        setPower(0.0)
    }

    /**
     * Sets a target angle and reassigns the [targetRotation] value so it can be called in [periodic]
     * Clears the PIDF total error for better position tracking
     * @param target the desired target angle
     */
    fun setTargetAngle(target: Angle) {
        targetRotation = target / config.gearRatio
    }

    /**
     * Changes the [targetRotation] considering gear ratios and the requested [change]
     * For example, if a negative 360 deg rotation is needed, the [changeTargetAngle] method does the job for getting
     * the target in terms of the servo.
     * @param change the desired change in the target rotation
     */
    fun changeTargetAngle(change: Angle) {
        targetRotation += (change / config.gearRatio)
    }

    /**
     * Manually sets a PIDF position tolerance
     * @param tolerance the desired position tolerance
     */
    fun setPIDFTolerance(tolerance: Angle) {
        pidfController.setTolerance(tolerance.degrees)
    }

    /**
     * Manually sets a new [PIDFCoefficients] to the servo's controller
     */
    fun setPIDF(pidfCoefficients: PIDFCoefficients) {
        pidfController.setPIDF(pidfCoefficients)
    }

    fun hadPIDFCoefficientsUpdated(pidfCoefficients: PIDFCoefficients): Boolean {
        return (pidfController.getCoefficients() == pidfCoefficients).not()
    }

    fun getPIDF(): PIDFCoefficients {
        return pidfController.getCoefficients()
    }

    /**
     * Gets the absolute position of the servo, not considering gear ratios
     */
    private fun getServoAbsoluteAngle(): Angle {
        val currentAngle = Angle.fromDegrees(
            (servoEncoder.voltage / config.absoluteMaxVoltage.volts) * (if (config.direction == Direction.REVERSE) -360 else 360)
        )
        val transformedAngle = Angle.fromDegrees(
            (currentAngle.degrees - config.encoderOffset.degrees)
        )

        return Angle.fromDegrees(normalizeDegrees(transformedAngle.degrees))
    }

    /**
     * Based on how many full rotations the servo has achieved, it returns an absolute angle
     * @return the total rotation the servo has achieved while considering gear ratios.
     */
    fun getAngle(): Angle {
        val transformedAngle = totalRotation.degrees * config.gearRatio
        val normalizedAngle = normalizeDegrees(transformedAngle)
        return Angle.fromDegrees(normalizedAngle)
    }

    fun getPIDFCoefficients(): PIDFCoefficients {
        return pidfController.getCoefficients()
    }

    /**
     * @return true is PID is at set point, false if not
     */
    fun isAtSetPoint(): Boolean {
        return pidfController.atSetpoint()
    }

    /**
     * Logs the servo's useful data
     */
    fun log(telemetry: TelemetryManager) {
        // Servo absolute angle -180° to 180°
        telemetry.addData("Current Servo Absolute Angle", getServoAbsoluteAngle().degrees)
        // Total of rotations the servo has achieved
        telemetry.addData("Complete rotations", fullRotations)
        // Encoder's voltage (useful when debugging)
        telemetry.addData("Absolute Encoder Voltage", servoEncoder.voltage)
        // If the servo is at set point
        telemetry.addData("Is at set point", isAtSetPoint())
        // System's angle (considering gear ratios)
        telemetry.addData("Absolute Angle (considering gear ratios)", getAngle().degrees)

        telemetry.addData("PIDF Coefficients", getPIDF().toString())
    }

    /**
     * Updates the servo's PID, and calculates the achieved full rotations
     * Necessary for the servo's correct functioning
     */
    fun periodic() {
        // Retrieves the current servo angle
        val currentAngle = getServoAbsoluteAngle()
        // Calculates the difference between the current and past servo angle
        val angleDifference = Angle.fromDegrees(currentAngle.degrees - previousAngle.degrees)

        // Calculates whether the servo has achieved one rotation, when the angle difference is below or above
        // 360 degrees and updates the full rotations
        if (angleDifference.degrees > Angle.fromDegrees(180.0).degrees) {
            fullRotations--
        } else if (angleDifference.degrees < Angle.fromDegrees(-180.0).degrees) {
            fullRotations++
        }

        // Calculates the total rotation considering the absolute angle and the achieved rotations
        totalRotation = currentAngle + Angle.fromDegrees(fullRotations * 360.0)

        // keeps track of the previous servo angle
        previousAngle = currentAngle

        // PID control
        val output = pidfController.calculate(totalRotation.degrees, targetRotation.degrees)

        // If at set point, no power is requested.
        if (isAtSetPoint().not()) {
            setPower(output)
        }
    }
}