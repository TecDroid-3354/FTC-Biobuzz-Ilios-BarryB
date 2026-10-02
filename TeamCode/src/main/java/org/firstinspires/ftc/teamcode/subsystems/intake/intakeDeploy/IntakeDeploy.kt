package org.firstinspires.ftc.teamcode.subsystems.intake.intakeDeploy

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.SubsystemBase
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
        configureServos()
    }

    override fun periodic() {
        if (leadIntakeDeployServo.hadRTPCoefficientsUpdated(SubsystemControlGains.INTAKE_DEPLOY_SERVOS_PIDF)) {
            updateIntakeDeployPIDF(SubsystemControlGains.INTAKE_DEPLOY_SERVOS_PIDF)
        }
    }

    private fun updateIntakeDeployPIDF(pidfCoefficients: PIDFCoefficients) {
        leadIntakeDeployServo.updateRunToPositionPIDF(pidfCoefficients)
        followerIntakeDeployServo.updateRunToPositionPIDF(pidfCoefficients)
    }

    private fun setIntakeDeployAngle(angle: Angle): Runnable {
        return {
            intakeDeployTargetAngle = angle.coerceIn(SubsystemLimits.INTAKE_DEPLOY_ANGLE_LIMITS)

            leadIntakeDeployServo.runToPosition(angle)
            followerIntakeDeployServo.runToPosition(angle)
        }
    }

    fun setIntakeDeployFloorAngle(): Command {
        return setIntakeDeployAngle(SubsystemPresetTargets.INTAKE_DEPLOY_FLOOR_ANGLE).InstantCommand(this)
    }

    fun setIntakeDeployFlowerAngle(): Command {
        return setIntakeDeployAngle(SubsystemPresetTargets.INTAKE_DEPLOY_FLOWER_ANGLE).InstantCommand(this)
    }

    fun setIntakeDeployConfigurableAngle(): Command {
        return setIntakeDeployAngle(Angle.fromDegrees(SubsystemConfigurableTargets.INTAKE_DEPLOY_CONFIGURABLE_DEGREES)).InstantCommand(this)
    }

    fun getIsAtTarget(): Boolean {
        return abs(
            (intakeDeployTargetAngle.minus(leadIntakeDeployServo.getAngle())).degrees
        ) < SubsystemTolerances.INTAKE_DEPLOY_ANGLE_TOLERANCE.degrees
    }

    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Intake Deploy")
        telemetry.addData("Intake Deploy Lead Servo Angle Degrees", leadIntakeDeployServo.getAngle().degrees)
        telemetry.addData("Intake Deploy Follower Servo Angle Degrees", followerIntakeDeployServo.getAngle().degrees)
        telemetry.addData("Intake Deploy Target Angle Degrees", intakeDeployTargetAngle.degrees)
    }

    private fun configureServos() {
        leadIntakeDeployServo = OpServoEx(hardwareMap, IntakeDeployConstants.Identification.INTAKE_DEPLOY_LEAD_SERVO_ID)
        leadIntakeDeployServo.applyConfiguration(IntakeDeployConstants.Configuration.leadServoConfiguration)

        followerIntakeDeployServo = OpServoEx(hardwareMap, IntakeDeployConstants.Identification.INTAKE_DEPLOY_FOLLOWER_SERVO_ID)
        followerIntakeDeployServo.applyConfiguration(IntakeDeployConstants.Configuration.followerServoConfiguration)
    }
}