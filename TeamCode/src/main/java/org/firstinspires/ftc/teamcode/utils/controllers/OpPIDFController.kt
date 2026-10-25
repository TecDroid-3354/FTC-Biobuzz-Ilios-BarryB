package org.firstinspires.ftc.teamcode.utils.controllers

import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * @param kP Proportional gain.
 * @param kI Integral gain.
 * @param kD Derivative gain.
 * @param kF Feedforward gain (not part of WPI's PIDController — see class doc).
 * @param period The controller's update period, in seconds. WPI defaults this to 0.02 (20ms).
 */
class OpPIDFController(
    var kP: Double,
    var kI: Double,
    var kD: Double,
    var kF: Double = 0.0,
    private val period: Double = 0.02
) {

    init {
        require(period > 0.0) { "Controller period must be a positive number, got $period." }
    }

    constructor(): this(0.0, 0.0, 0.0, 0.0)

    constructor(pidfCoefficients: PIDFCoefficients): this(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f)

    // ---- Continuous input ----
    private var continuous = false
    private var minimumInput = 0.0
    private var maximumInput = 0.0

    // ---- Integrator ----
    private var minimumIntegral = -1.0
    private var maximumIntegral = 1.0
    private var iZone = Double.POSITIVE_INFINITY

    // ---- Setpoint / measurement / error state ----
    private var setpoint = 0.0
    private var measurement = 0.0
    private var positionError = 0.0
    private var velocityError = 0.0
    private var prevError = 0.0
    private var totalError = 0.0

    // ---- Tolerances ----
    private var positionTolerance = Double.POSITIVE_INFINITY
    private var velocityTolerance = Double.POSITIVE_INFINITY

    // ============================================================
    // Gains
    // ============================================================

    fun setPIDF(pidfCoefficients: PIDFCoefficients) {
        this.kP = pidfCoefficients.p
        this.kI = pidfCoefficients.i
        this.kD = pidfCoefficients.d
        this.kF = pidfCoefficients.f
    }

    /** Sets the PID gains in one call. */
    fun setPID(kP: Double, kI: Double, kD: Double) {
        this.kP = kP
        this.kI = kI
        this.kD = kD
    }

    fun setP(kP: Double) { this.kP = kP }
    fun setI(kI: Double) { this.kI = kI }
    fun setD(kD: Double) { this.kD = kD }
    fun setF(kF: Double) { this.kF = kF }

    fun getP(): Double = kP
    fun getI(): Double = kI
    fun getD(): Double = kD
    fun getF(): Double = kF
    fun getCoefficients(): PIDFCoefficients = PIDFCoefficients(kP, kI, kD, kF)
    fun getPeriod(): Double = period

    // ============================================================
    // Setpoint
    // ============================================================

    /**
     * Sets the setpoint for the controller.
     *
     * @param setpoint The desired setpoint, in the same units as the measurement
     * passed to [calculate].
     */
    fun setSetpoint(setpoint: Double) {
        this.setpoint = setpoint
    }

    fun getSetpoint(): Double = setpoint

    /**
     * Returns true if the error is within the tolerances set by [setTolerance].
     *
     * This will return false until at least one call to [setTolerance] has been
     * made, since the default tolerances are infinite.
     */
    fun atSetpoint(): Boolean {
        return abs(positionError) < positionTolerance && abs(velocityError) < velocityTolerance
    }

    // ============================================================
    // Continuous input
    // ============================================================

    /**
     * Enables continuous input, wrapping error around the min/max input range
     * (e.g. for a mechanism where 0 and [maximumInput] represent the same
     * physical position, like a full-rotation turret).
     */
    fun enableContinuousInput(minimumInput: Double, maximumInput: Double) {
        continuous = true
        this.minimumInput = minimumInput
        this.maximumInput = maximumInput
    }

    fun disableContinuousInput() {
        continuous = false
    }

    fun isContinuousInputEnabled(): Boolean = continuous

    // ============================================================
    // Integrator range / IZone
    // ============================================================

    /**
     * Sets the minimum and maximum contributions of the integral term (i.e.
     * the output is clamped so `kI * totalError` stays within this range).
     * Defaults to [-1.0, 1.0].
     */
    fun setIntegratorRange(minimumIntegral: Double, maximumIntegral: Double) {
        this.minimumIntegral = minimumIntegral
        this.maximumIntegral = maximumIntegral
    }

    /**
     * Sets the error range ("IZone") outside of which the integral term is
     * disabled and the accumulated error is reset to zero. Set to
     * [Double.POSITIVE_INFINITY] (the default) to disable this feature.
     */
    fun setIZone(iZone: Double) {
        require(iZone >= 0.0) { "IZone must be a non-negative number, got $iZone." }
        this.iZone = iZone
    }

    fun getIZone(): Double = iZone

    // ============================================================
    // Tolerances
    // ============================================================

    /**
     * Sets the error tolerance used by [atSetpoint]. If [velocityTolerance]
     * is omitted it defaults to infinity, matching WPI's behavior.
     */
    fun setTolerance(positionTolerance: Double, velocityTolerance: Double = Double.POSITIVE_INFINITY) {
        this.positionTolerance = positionTolerance
        this.velocityTolerance = velocityTolerance
    }

    // ============================================================
    // Error accessors
    // ============================================================

    fun getPositionError(): Double = positionError
    fun getVelocityError(): Double = velocityError

    // ============================================================
    // Calculate
    // ============================================================

    /**
     * Calculates the controller output.
     *
     * @param measurement The current measurement (e.g. the servo's absolute
     * encoder position).
     * @param feedforward Optional static feedforward term, multiplied by [kF]
     * and added to the output. Defaults to 0.0, which makes this behave
     * identically to WPI's `PIDController.calculate(measurement)`.
     */
    fun calculate(measurement: Double, feedforward: Double = 0.0): Double {
        this.measurement = measurement
        prevError = positionError

        positionError = if (continuous) {
            val errorBound = (maximumInput - minimumInput) / 2.0
            inputModulus(setpoint - measurement, -errorBound, errorBound)
        } else {
            setpoint - measurement
        }

        velocityError = (positionError - prevError) / period

        if (abs(positionError) > iZone) {
            totalError = 0.0
        } else if (kI != 0.0) {
            totalError = clamp(
                totalError + positionError * period,
                minimumIntegral / kI,
                maximumIntegral / kI
            )
        }

        return kP * positionError + kI * totalError + kD * velocityError + kF * feedforward
    }

    /**
     * Sets the setpoint and calculates the controller output in one call.
     */
    fun calculate(measurement: Double, setpoint: Double, feedforward: Double = 0.0): Double {
        this.setpoint = setpoint
        return calculate(measurement, feedforward)
    }

    // ============================================================
    // Reset
    // ============================================================

    /**
     * Resets the controller's internal state (most importantly the integral
     * accumulator). Call this whenever the loop was disabled and is about to
     * be re-enabled, so old accumulated error doesn't cause a jump in output.
     */
    fun reset() {
        positionError = 0.0
        prevError = 0.0
        totalError = 0.0
        velocityError = 0.0
    }

    // ============================================================
    // MathUtil
    // ============================================================

    private companion object {
        /**
         * Direct port of WPI's `MathUtil.clamp(double, double, double)`.
         */
        fun clamp(value: Double, low: Double, high: Double): Double {
            return max(low, min(value, high))
        }

        /**
         * Direct port of WPI's `MathUtil.inputModulus(double, double, double)`,
         * used to wrap error onto the shortest path when continuous input is
         * enabled.
         */
        fun inputModulus(inputValue: Double, minimumInput: Double, maximumInput: Double): Double {
            var input = inputValue
            val modulus = maximumInput - minimumInput

            // Wrap input if it's above the maximum input
            val numMax = ((input - minimumInput) / modulus).toInt()
            input -= numMax * modulus

            // Wrap input if it's below the minimum input
            val numMin = ((input - maximumInput) / modulus).toInt()
            input -= numMin * modulus

            return input
        }
    }
}

fun PIDCoefficients.invertSigns(): PIDCoefficients {
    return PIDCoefficients(this.p.unaryMinus(), this.i.unaryMinus(), this.d.unaryMinus())
}

fun SimpleMotorFeedforward.invertSigns(): SimpleMotorFeedforward {
    return SimpleMotorFeedforward(this.ks.unaryMinus(), this.kv.unaryMinus(), this.ka.unaryMinus())
}
