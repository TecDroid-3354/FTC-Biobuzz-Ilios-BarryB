package org.firstinspires.ftc.teamcode.utils.controllers

import kotlin.math.abs

/**
 * Implements a bang-bang controller, which outputs either 0 or 1 depending on whether the
 * measurement is less than the setpoint. This maximally-aggressive control approach works very
 * well for velocity control of high-inertia mechanisms, and poorly on most other things.
 *
 * Ported from WPILib's `edu.wpi.first.math.controller.BangBangController`
 *
 * **Note:** Do not use this controller with a motor in brake mode. Use coast
 * (`ZeroPowerBehavior.FLOAT`) and raw power (`Motor.RunMode.RawPower`).
 * Brake mode actively fights the flywheel when the output drops to 0 and will cause
 * large oscillations and mechanical stress.
 *
 * Since this is a one-sided (0 or 1) controller, the output is in the range [0, 1] (motor power).
 * Combine it with a slightly-under-nominal feedforward to avoid overshoot at the setpoint.
 */
class BangBangController(tolerance: Double = Double.POSITIVE_INFINITY) {

    private var toleranceValue: Double = Double.POSITIVE_INFINITY
    private var setpointValue: Double = 0.0
    private var measurementValue: Double = 0.0

    init {
        setTolerance(tolerance)
    }

    /**
     * Sets the setpoint for the bang-bang controller.
     *
     * @param setpoint The desired setpoint.
     */
    fun setSetpoint(setpoint: Double) {
        this.setpointValue = setpoint
    }

    /**
     * Returns the current setpoint of the bang-bang controller.
     *
     * @return The current setpoint.
     */
    fun getSetpoint(): Double {
        return setpointValue
    }

    /**
     * Returns true if the error is within the tolerance of the setpoint.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    fun atSetpoint(): Boolean {
        return abs(setpointValue - measurementValue) < toleranceValue
    }

    /**
     * Sets the error within which [atSetpoint] will return true.
     *
     * @param tolerance Position error which is tolerable.
     */
    fun setTolerance(tolerance: Double) {
        this.toleranceValue = tolerance
    }

    /**
     * Returns the current tolerance of the controller.
     *
     * @return The current tolerance.
     */
    fun getTolerance(): Double {
        return toleranceValue
    }

    /**
     * Returns the current measurement of the process variable.
     *
     * @return The current measurement of the process variable.
     */
    fun getMeasurement(): Double {
        return measurementValue
    }

    /**
     * Returns the current error.
     *
     * @return The current error.
     */
    fun getError(): Double {
        return setpointValue - measurementValue
    }

    /**
     * Returns the calculated control output.
     *
     * @param measurement The most recent measurement of the process variable.
     * @param setpoint The setpoint for the process variable.
     * @return 1.0 if the measurement is below the setpoint, otherwise 0.0.
     */
    fun calculate(measurement: Double, setpoint: Double): Double {
        this.measurementValue = measurement
        this.setpointValue = setpoint

        return if (measurement < setpoint) 1.0 else 0.0
    }

    /**
     * Returns the calculated control output using the previously stored setpoint.
     *
     * @param measurement The most recent measurement of the process variable.
     * @return 1.0 if the measurement is below the setpoint, otherwise 0.0.
     */
    fun calculate(measurement: Double): Double {
        return calculate(measurement, setpointValue)
    }
}