package org.firstinspires.ftc.teamcode.subsystems.intake.intakeDeploy

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.constants.SubsystemConfigurableTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemLimits
import org.firstinspires.ftc.teamcode.constants.SubsystemPresetTargets
import org.firstinspires.ftc.teamcode.constants.SubsystemTolerances
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx
import org.firstinspires.ftc.teamcode.utils.units.Angle
import kotlin.math.abs

class FlowerDropDown(private val hardwareMap: HardwareMap): SubsystemBase() {

    private lateinit var leadDropDownServo: OpServoEx

    private lateinit var followerDropDownServo: OpServoEx

    private var flowerDropDownTargetAngle: Angle = Angle.fromDegrees(0.0)

    private var configurableFlowerDropDownAngle: Angle = Angle(0.0)

    init {
        // Initialization code
        configureServos()

        setFlowerDropDownFloorAngle().run()
    }

    /** Runs every cycle */
    override fun periodic() {
        if (configurableFlowerDropDownAngle.degrees.equals(SubsystemConfigurableTargets.INTAKE_DEPLOY_CONFIGURABLE_DEGREES).not()) {
            updateFlowerDropDownConfigurableAngle(SubsystemConfigurableTargets.INTAKE_DEPLOY_CONFIGURABLE_DEGREES)
        }
    }

    /** Updates [configurableFlowerDropDownAngle] with the value of [SubsystemConfigurableTargets.INTAKE_DEPLOY_CONFIGURABLE_DEGREES]*/
    private fun updateFlowerDropDownConfigurableAngle(newFlowerDropDownConfigurableAngleDegrees: Double) {
        configurableFlowerDropDownAngle = Angle.fromDegrees(newFlowerDropDownConfigurableAngleDegrees)
    }

    /**
     * Sets the floor intake angle to [SubsystemPresetTargets.DROP_DOWN_FLOOR_ANGLE] value.
     * @return a [Runnable] which sets a new angle.
     */
    fun setFlowerDropDownFloorAngle(): Runnable {
        return {
            flowerDropDownTargetAngle = SubsystemPresetTargets.DROP_DOWN_FLOOR_ANGLE

            leadDropDownServo.setServoPosition(
                SubsystemPresetTargets.DROP_DOWN_FLOOR_ANGLE,
                SubsystemLimits.DROP_DOWN_ANGLE_LIMITS,
                FlowerDropDownConstants.Mechanical.GEAR_RATIO
            )
            followerDropDownServo.setServoPosition(
                SubsystemPresetTargets.DROP_DOWN_FLOOR_ANGLE,
                SubsystemLimits.DROP_DOWN_ANGLE_LIMITS,
                FlowerDropDownConstants.Mechanical.GEAR_RATIO
            )
        }
    }

    /**
     * Sets the flower intake angle to [SubsystemPresetTargets.DROP_DOWN_FLOWER_ANGLE] value.
     * @return a [Runnable] which sets a new angle.
     */
    fun setFlowerDropDownFlowerAngle(): Runnable {
        return {
            flowerDropDownTargetAngle = SubsystemPresetTargets.DROP_DOWN_FLOWER_ANGLE

            leadDropDownServo.setServoPosition(
                SubsystemPresetTargets.DROP_DOWN_FLOWER_ANGLE,
                SubsystemLimits.DROP_DOWN_ANGLE_LIMITS,
                FlowerDropDownConstants.Mechanical.GEAR_RATIO
            )
            followerDropDownServo.setServoPosition(
                SubsystemPresetTargets.DROP_DOWN_FLOWER_ANGLE,
                SubsystemLimits.DROP_DOWN_ANGLE_LIMITS,
                FlowerDropDownConstants.Mechanical.GEAR_RATIO
            )
        }
    }

    /**
     * Sets the intake deploy angle to [SubsystemConfigurableTargets.INTAKE_DEPLOY_CONFIGURABLE_DEGREES] value.
     * @return a [Runnable] which sets a new angle.
     */
    fun setFlowerDropDownConfigurableAngle(): Runnable {
        return {
            val clampedAngleDegrees = MathUtils.clamp(configurableFlowerDropDownAngle.degrees,
                SubsystemLimits.DROP_DOWN_ANGLE_LIMITS.start.degrees, SubsystemLimits.DROP_DOWN_ANGLE_LIMITS.endInclusive.degrees)
            flowerDropDownTargetAngle = Angle.fromDegrees(clampedAngleDegrees)

            leadDropDownServo.setServoPosition(
                flowerDropDownTargetAngle,
                SubsystemLimits.DROP_DOWN_ANGLE_LIMITS,
                FlowerDropDownConstants.Mechanical.GEAR_RATIO
            )
            followerDropDownServo.setServoPosition(
                flowerDropDownTargetAngle,
                SubsystemLimits.DROP_DOWN_ANGLE_LIMITS,
                FlowerDropDownConstants.Mechanical.GEAR_RATIO
            )
        }
    }

    /**
     * @return whether the [FlowerDropDown] has reached its angle target.
     */
    fun getIsAtTarget(): Boolean {
        return abs(
            (flowerDropDownTargetAngle.degrees.minus(leadDropDownServo.getRawPosition() * FlowerDropDownConstants.Configuration.servoRange.endInclusive.degrees))
        ) < SubsystemTolerances.DROP_DOWN_ANGLE_TOLERANCE.degrees
    }

    // Valuable indexer information, call inside Robot.printTelelmetry()
    fun log(telemetry: TelemetryManager) {
        telemetry.addLine("Intake Deploy")
        telemetry.addData("Intake Deploy Angle Degrees", leadDropDownServo.getRawPosition() * FlowerDropDownConstants.Configuration.servoRange.endInclusive.degrees)
        telemetry.addData("Intake Deploy Target Angle Degrees", flowerDropDownTargetAngle.degrees)
    }

    // Initialize both servos
    private fun configureServos() {
        leadDropDownServo = OpServoEx(hardwareMap, FlowerDropDownConstants.Identification.DROP_DOWN_LEAD_SERVO_ID)
        leadDropDownServo.applyConfiguration(FlowerDropDownConstants.Configuration.leadServoConfiguration)

        followerDropDownServo = OpServoEx(hardwareMap, FlowerDropDownConstants.Identification.DROP_DOWN_FOLLOWER_SERVO_ID)
        followerDropDownServo.applyConfiguration(FlowerDropDownConstants.Configuration.followerServoConfiguration)
    }
}