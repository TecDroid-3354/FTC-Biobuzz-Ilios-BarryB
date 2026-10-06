package org.firstinspires.ftc.teamcode.subsystems.intake.intakeDeploy

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemControlGains
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx
import org.firstinspires.ftc.teamcode.utils.extensions.InstantCommand
import org.firstinspires.ftc.teamcode.utils.units.Angle
import kotlin.math.abs

class IntakeDeploy(private val hardwareMap: HardwareMap): SubsystemBase() {

    private lateinit var leadIntakeDeployServo: OpServoEx

    private lateinit var followerIntakeDeployServo: OpServoEx

    private var intakeDeployTargetAngle: Angle = Angle(0.0)

    init {
        // Initialization code
        configureServos()
    }

    /** Runs every cycle */
    override fun periodic() {
        if (leadIntakeDeployServo.hadRTPCoefficientsUpdated(SubsystemControlGains.INTAKE_DEPLOY_SERVOS_PIDF)) {
            updateIntakeDeployPIDF(SubsystemControlGains.INTAKE_DEPLOY_SERVOS_PIDF)
        }
    }

    /**
     * Updates the [leadIntakeDeployServo] and [followerIntakeDeployServo]s [com.qualcomm.robotcore.hardware.PIDCoefficients] coefficients
     * by calling [OpServoEx.updateRunToPositionPIDF]
     * @param pidfCoefficients the new [PIDCoefficients]
     */
    private fun updateIntakeDeployPIDF(pidfCoefficients: PIDFCoefficients) {
        leadIntakeDeployServo.updateRunToPositionPIDF(pidfCoefficients)
        followerIntakeDeployServo.updateRunToPositionPIDF(pidfCoefficients)
    }

    /**
     * A [Runnable] which sets a new angle to the [IntakeDeploy] with the given [Angle] after being clamped
     * by the [SubsystemLimits.INTAKE_DEPLOY_ANGLE_LIMITS].
     * @param angle the desired [Angle]
     * @return a [Runnable] which sets the new angle
     */
    private fun setIntakeDeployAngle(angle: Angle): Runnable {
        return {
            intakeDeployTargetAngle = angle.coerceIn(SubsystemLimits.INTAKE_DEPLOY_ANGLE_LIMITS)

            leadIntakeDeployServo.runToPosition(angle)
            followerIntakeDeployServo.runToPosition(angle)
        }
    }

    /**
     * Calls [setIntakeDeployAngle] and sets the floor intake angle to [SubsystemPresetTargets.INTAKE_DEPLOY_FLOOR_ANGLE] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets a new angle.
     */
    fun setIntakeDeployFloorAngle(): Command {
        return setIntakeDeployAngle(SubsystemPresetTargets.INTAKE_DEPLOY_FLOOR_ANGLE).InstantCommand(this)
    }

    /**
     * Calls [setIntakeDeployAngle] and sets the flower intake angle to [SubsystemPresetTargets.INTAKE_DEPLOY_FLOWER_ANGLE] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets a new angle.
     */
    fun setIntakeDeployFlowerAngle(): Command {
        return setIntakeDeployAngle(SubsystemPresetTargets.INTAKE_DEPLOY_FLOWER_ANGLE).InstantCommand(this)
    }

    /**
     * Calls [setIntakeDeployAngle] and sets the intake deploy angle to [SubsystemConfigurableTargets.INTAKE_DEPLOY_CONFIGURABLE_DEGREES] value.
     * @return an [com.seattlesolvers.solverslib.command.InstantCommand] which sets a new angle.
     */
    fun setIntakeDeployConfigurableAngle(): Command {
        return setIntakeDeployAngle(Angle.fromDegrees(SubsystemConfigurableTargets.INTAKE_DEPLOY_CONFIGURABLE_DEGREES)).InstantCommand(this)
    }

    /**
     * @return whether the [IntakeDeploy] has reached its angle target.
     */
    fun getIsAtTarget(): Boolean {
        if (intakeDeployTargetAngle < IntakeDeployConstants.Mechanical.TARGET_ANGLE_THRESHOLD) return false

        return abs(
            (intakeDeployTargetAngle.minus(leadIntakeDeployServo.getAngle())).degrees
        ) < SubsystemTolerances.INTAKE_DEPLOY_ANGLE_TOLERANCE.degrees
    }

    // Valuable indexer information, call inside Robot.printTelelmetry()
    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Intake Deploy")
        telemetry.addData("Intake Deploy Lead Servo Angle Degrees", leadIntakeDeployServo.getAngle().degrees)
        telemetry.addData("Intake Deploy Follower Servo Angle Degrees", followerIntakeDeployServo.getAngle().degrees)
        telemetry.addData("Intake Deploy Target Angle Degrees", intakeDeployTargetAngle.degrees)
    }

    // Initialize both servos
    private fun configureServos() {
        leadIntakeDeployServo = OpServoEx(hardwareMap, IntakeDeployConstants.Identification.INTAKE_DEPLOY_LEAD_SERVO_ID)
        leadIntakeDeployServo.applyConfiguration(IntakeDeployConstants.Configuration.leadServoConfiguration)

        followerIntakeDeployServo = OpServoEx(hardwareMap, IntakeDeployConstants.Identification.INTAKE_DEPLOY_FOLLOWER_SERVO_ID)
        followerIntakeDeployServo.applyConfiguration(IntakeDeployConstants.Configuration.followerServoConfiguration)
    }
}