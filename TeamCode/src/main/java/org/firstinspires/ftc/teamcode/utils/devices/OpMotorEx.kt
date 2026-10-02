@file:Suppress("JoinDeclarationAndAssignment")

package org.firstinspires.ftc.teamcode.utils.devices

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.configurations.OpMotorExConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorControlModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorPercentageModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorTrapezoidalModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.controllers.TrapezoidalMotionProfile
import org.firstinspires.ftc.teamcode.utils.extensions.applyGenericConfiguration
import org.firstinspires.ftc.teamcode.utils.extensions.setVelocityCoefficients
import org.firstinspires.ftc.teamcode.utils.extensions.setVelocityFeedforward
import org.firstinspires.ftc.teamcode.utils.devices.configurations.genericConfigurations.GenericMotorConfiguration
import org.firstinspires.ftc.teamcode.utils.TecDroidRobot
import org.firstinspires.ftc.teamcode.utils.devices.controlModes.MotorControlMode
import java.util.function.BooleanSupplier
import java.util.function.Supplier
import kotlin.math.max
import kotlin.math.min

class OpMotorEx(hardwareMap: HardwareMap, motorId: String) {

    // ----- Motor and Control Mode variable's creation ----- //
    private var motor                           : MotorEx
    private var controlMode                     : MotorControlMode = MotorControlMode.UNKNOWN
    // ----- Useful variables, avoids repetition ----- //
    var reduction                               : Double = 1.0
    private var countPerRev                     : Double = 28.0
    private var maxPower                        : Double = 1.0
    private var maxVelocity                     : AngularVelocity = AngularVelocity(0.0)
    private var targetAngle                     : Angle = Angle(0.0)
    private lateinit var positionLimits         : ClosedRange<Angle>
    // ----- Position and trapezoidal controller declaration ----- //
    private lateinit var positionController     : PIDFController
    private lateinit var trapezoidalController  : TrapezoidalMotionProfile
    // ----- Commanding wrong control mode exception ----- //
    private val wrongControlModeCommandException: IllegalAccessError = IllegalAccessError(
        "Control mode not configured correctly, double check the Control Mode Configuration given"
    )

    // Initialization code. Runs at class init //
    init {
        motor = MotorEx(hardwareMap, motorId)
        motor.set(0.0)
        registry.add(this)
    }

    // ------ MOTOR MEASURES RETRIEVAL WITH GEAR RATIOS -----//
    /**
     * Constructs a new [Angle] value and returns it
     * @return the current motor's angle
     * Used ONLY for the trapezoidal motion profile
     */
    fun getPosition(): Supplier<Angle> { return { Angle(motor.currentPosition / countPerRev / reduction) } }
    /**
     * Constructs a new [AngularVelocity] value and returns it
     * @return the current motor's velocity
     */
    fun getVelocity(): Supplier<AngularVelocity> { return { AngularVelocity(motor.velocity / countPerRev / reduction) } }
    /**
     * @return the motor's set power
     */
    fun getOutput(): Double { return motor.get() }

    /**
     * Whether the motor is connected. If the Current drawn by the motor is less than 10 milliamps it will return true.
     * 10 milliamps is an arbitrary value
     * @return whether the motor is connected
     */
    fun getIsConnected(): BooleanSupplier { return { (motor.motorEx.getCurrent(CurrentUnit.MILLIAMPS) < 10.0).not() }}

    // ------ MOTOR CONTROL MODE COMMANDS -----//
    /**
     * Sets the motor's velocity to a percentage of its capability from -1.0 to 1.0.
     * Check if a [MotorPercentageModeConfiguration] was passed to the motor's configuration.
     * If not, an [IllegalAccessError] will be thrown.
     * @param power the desired motor's power from -1.0 to 1.0
     */
    fun setPower(power: Double) {
        if (controlMode != MotorControlMode.PERCENTAGE) { throw wrongControlModeCommandException }

        val clampedPower = max(-maxPower, min(maxPower, power))
        motor.set(clampedPower)
    }

    /**
     * Sets the motor's velocity to a desired [AngularVelocity]. Motor's [countPerRev] and [reduction]
     * are already taken into account to get the system's [maxVelocity].  Pass these values inside a [GenericMotorConfiguration].
     * Checks if a [MotorVelocityModeConfiguration] was passed to the motor's configuration.
     * If not, an [IllegalAccessError] will be thrown
     * @param velocity the desired angular velocity
     */
    fun setVelocity(velocity: AngularVelocity) {
        if (controlMode != MotorControlMode.VELOCITY) { throw wrongControlModeCommandException }

        val clampedVelocity = velocity.rps.coerceIn(maxVelocity.rps.unaryMinus(), maxVelocity.rps)
        val transformedVelocity = clampedVelocity * reduction
        val velocityInTicksPerSec = transformedVelocity * countPerRev

        motor.velocity = velocityInTicksPerSec
    }

    /**
     * Sets the motor's angle to a desired [Angle]. Motor's [countPerRev] and [reduction]
     * are already taken into account. Pass these values inside a [GenericMotorConfiguration].
     * Limits are also considered when passing them through a [MotorPositionModeConfiguration]
     * Checks if a [MotorPositionModeConfiguration] was passed to the motor's configuration.
     * If not, an [IllegalAccessError] will be thrown.
     * @param angle the desired angle
     */
    fun setAngle(angle: Angle) {
        if (controlMode != MotorControlMode.POSITION) { throw wrongControlModeCommandException }

        val clampedAngle = angle.rotations.coerceIn(positionLimits.start.rotations, positionLimits.endInclusive.rotations)
        targetAngle = Angle(clampedAngle)
    }

    /**
     * Sets the motor's angle to a desired [Angle] by profiling the motor's movement through [TrapezoidalMotionProfile].
     * Motor's [countPerRev] and [reduction] are already taken into account. Pass these values inside a [GenericMotorConfiguration].
     * Limits are also considered when passing them through a [MotorTrapezoidalModeConfiguration]
     * Checks if a [MotorTrapezoidalModeConfiguration] was passed to the motor's configuration.
     * If not, an [IllegalAccessError] will be thrown.
     * @param angle the desired profiled angle
     */
    fun setProfiledAngle(angle: Angle) {
        if (controlMode != MotorControlMode.TRAPEZOIDAL) { throw wrongControlModeCommandException }

        val clampedAngle = angle.rotations.coerceIn(positionLimits.start.rotations, positionLimits.endInclusive.rotations)
        targetAngle = Angle(clampedAngle)
        trapezoidalController.setTargetAngle(targetAngle)
    }

    /**
     * Stops the motor by calling its set method and assigning a new value of 0.0. When stopping a motor do not
     * call the built-in [com.seattlesolvers.solverslib.hardware.motors.Motor.stopMotor] as it interferes with the velocity
     * mode's commands.
     */
    fun stopMotor(): Runnable { return { motor.set(0.0) } }

    /**
     * Checks whether the [positionController] or the [trapezoidalController]'s position is near its target or at it
     * based on the tolerance declared in its respective [MotorControlModeConfiguration]. If the configured mode does not correpond
     * to either positon or trapezoidal an error will be thrown.
     * @return whether the controller has reached its target position
     */
    fun isAtPositionTarget(): BooleanSupplier {
        return {
            when (controlMode) {
                MotorControlMode.POSITION -> positionController.atSetPoint()
                MotorControlMode.TRAPEZOIDAL -> trapezoidalController.atSetPoint()
                else -> throw wrongControlModeCommandException
            }
        }
    }

    /**
     * @return this [OpMotorEx] correspondent [MotorEx] instance
     */
    fun getMotorInstance(): MotorEx { return motor }

    // --------- PID and Feedforward configurables update -------- //

    fun hadVelocityPIDControlGainsUpdated(pidCoefficients: PIDCoefficients): Boolean {
        if (controlMode != MotorControlMode.VELOCITY) { throw wrongControlModeCommandException }

        val kP = motor.veloCoefficients[0]
        val kI = motor.veloCoefficients[1]
        val kD = motor.veloCoefficients[2]

        return kP.equals(pidCoefficients.p).not() || kI.equals(pidCoefficients.i).not() || kD.equals(pidCoefficients.d).not()
    }

    fun hadVelocityFeedforwardControlGainsUpdated(feedforward: SimpleMotorFeedforward): Boolean {
        if (controlMode != MotorControlMode.VELOCITY) { throw wrongControlModeCommandException }

        val kS = motor.feedforwardCoefficients[0]
        val kV = motor.feedforwardCoefficients[1]
        val kA = motor.feedforwardCoefficients[2]

        return kS.equals(feedforward.ks).not() || kV.equals(feedforward.kv).not() || kA.equals(feedforward.ka).not()
    }

    fun hadPIDFPositionControlGainsUpdated(pidfCoefficients: PIDFCoefficients): Boolean {
        if (controlMode != MotorControlMode.POSITION) { throw wrongControlModeCommandException }

        val kP = positionController.p
        val kI = positionController.i
        val kD = positionController.d
        val kF = positionController.f

        return kP.equals(pidfCoefficients.p).not() || kI.equals(pidfCoefficients.i).not() ||
                kD.equals(pidfCoefficients.d).not() || kF.equals(pidfCoefficients.f).not()
    }

    fun hadTrapezoidalControlGainsUpdated(pidCoefficients: PIDCoefficients): Boolean {
        if (controlMode != MotorControlMode.TRAPEZOIDAL) { throw wrongControlModeCommandException }

        val kP = trapezoidalController.getPID().p
        val kI = trapezoidalController.getPID().i
        val kD = trapezoidalController.getPID().d

        return kP.equals(pidCoefficients.p).not() || kI.equals(pidCoefficients.i).not() || kD.equals(pidCoefficients.d).not()
    }

    /**
     * Per-instance update logic. Called automatically for every existing [OpMotorEx] inside [TecDroidRobot]
     * when [updateAll] is invoked from its main loop. There is no need to call this method directly from a subsystem.
     * Updates the [trapezoidalController] or [positionController] if this [OpMotorEx] instance was initialized either
     * wtih [MotorControlMode.TRAPEZOIDAL] or [MotorControlMode.POSITION]
     * Checks if [controlMode] has already been initialized when configuring the motor
     * throguh [applyConfigurationAndResetEncoder]. If not, method's skipped
     */
    private fun updateMotor() {
        if (controlMode == MotorControlMode.UNKNOWN) return

        when (controlMode) {
            MotorControlMode.POSITION -> {
                val power = positionController.calculate(getPosition().get().rotations, targetAngle.rotations)
                motor.set(power)
            }
            MotorControlMode.TRAPEZOIDAL -> {
                trapezoidalController.updateProfile()
            }
            else -> { /* Nothing's done here as the rest of control modes do not require an update in their respective controllers */ }
        }
    }

    /**
     * Applies a new [MotorControlModeConfiguration] to this [OpMotorEx].
     * This method MUST NOT be called before the original [applyConfigurationAndResetEncoder]
     * If this configuration's [MotorControlMode] is different from the last one, some methods
     * won't be accessible once this config is applied.
     * Double check that you're passing the correct [MotorControlModeConfiguration].
     * @param config the new [MotorControlModeConfiguration]
     */
    fun applyModeConfiguration(config: MotorControlModeConfiguration) {
        if (config.controlMode != controlMode) {
            return
        }

        when (config) {
            is MotorPercentageModeConfiguration -> {
                maxPower = config.maxPower
            }
            is MotorVelocityModeConfiguration -> {
                motor.setVelocityCoefficients(config.velocityCoefficients)
                motor.setVelocityFeedforward(config.feedforwardCoefficients)
            }
            is MotorPositionModeConfiguration -> {
                positionController = PIDFController(config.positionCoefficients)
                positionController.setTolerance(config.positionTolerance)
                positionLimits = config.positionLimits
            }
            is MotorTrapezoidalModeConfiguration -> {
                trapezoidalController = TrapezoidalMotionProfile(this, config)
                trapezoidalController.setPositionTolerance(config.positionTolerance)
                positionLimits = config.profileLimits
            }
        }
    }

    /**
     * Applies a new [OpMotorExConfiguration]. This method MUST be called before using any of the control methods
     * to ensure correct working. [IllegalAccessException] may be thrown if not.
     * The [OpMotorEx]'s [MotorControlMode] will be automatically applied depending on the [MotorControlModeConfiguration]
     * bounded to the [OpMotorExConfiguration].
     * The [GenericMotorConfiguration] should also be passed in as motor inversions and measured values depend
     * on this configuration.
     * The motor's stopped after being configured and its encoder is zeroed
     * @param newConfig the desired [OpMotorExConfiguration] to apply
     */
    fun applyConfigurationAndResetEncoder(newConfig: OpMotorExConfiguration) {
        countPerRev = newConfig.genericMotorConfiguration.ticksPerRev
        reduction = newConfig.genericMotorConfiguration.reduction
        controlMode = newConfig.controlModeConfiguration.controlMode
        maxVelocity = AngularVelocity(100.0 / reduction)
        motor.applyGenericConfiguration(newConfig.genericMotorConfiguration, controlMode)

        when (val modeConfig = newConfig.controlModeConfiguration) {
            is MotorPercentageModeConfiguration -> {
                maxPower = modeConfig.maxPower
            }
            is MotorVelocityModeConfiguration -> {
                motor.setVelocityCoefficients(modeConfig.velocityCoefficients)
                motor.setVelocityFeedforward(modeConfig.feedforwardCoefficients)
            }
            is MotorPositionModeConfiguration -> {
                positionController = PIDFController(modeConfig.positionCoefficients)
                positionController.setTolerance(modeConfig.positionTolerance)
                positionLimits = modeConfig.positionLimits
            }
            is MotorTrapezoidalModeConfiguration -> {
                trapezoidalController = TrapezoidalMotionProfile(this, modeConfig)
                trapezoidalController.setPositionTolerance(modeConfig.positionTolerance)
                positionLimits = modeConfig.profileLimits
            }
        }
        motor.stopAndResetEncoder()
    }

    /**
     * Companion objects belong to the class at top-level, meaning there's just one companion
     * per class creation, not class's instances.
     */
    companion object {
        private val registry = mutableListOf<OpMotorEx>()

        /**
         * Calls [updateMotor] on every [OpMotorEx] instance that currently exists.
         * Called from [TecDroidRobot] run method to ensure its call no matter what.
         */
        fun updateAll() {
            registry.forEach { it.updateMotor() }
        }

        /**
         * Clears the registry of all tracked instances. MUST be called once in the
         * [TecDroidRobot]'s init block, before any [OpMotorEx] (or subsystem that creates one)
         * is constructed.
         * Prevents that motors created on previous OpMode are updated in a new one.
         */
        fun clearRegistry() {
            registry.clear()
        }
    }
}