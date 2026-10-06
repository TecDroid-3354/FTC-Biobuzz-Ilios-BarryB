package org.firstinspires.ftc.teamcode.constants

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants
import org.firstinspires.ftc.teamcode.subsystems.intake.intakeRollers.IntakeRollersConstants
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelConstants
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity

object DriveMultipliers {
    const val FORWARD_VELOCITY_MULTIPLIER           : Double    = 1.0
    const val LATERAL_VELOCITY_MULTIPLIER           : Double    = 1.0
    const val TURN_VELOCITY_MULTIPLIER              : Double    = -1.0
    const val CONTROLLER_SOTM_LINEAR_MULTIPLIER     : Double    = 0.5
    const val CONTROLLER_SOTM_ANGULAR_MULTIPLIER    : Double    = 0.6
}

object SubsystemTolerances {
    val FLYWHEEL_RPM_TOLERANCE = AngularVelocity.fromRpm(40.0)
    val FLYWHEEL_HARD_STOP_ANGLE_TOLERANCE = Angle.fromDegrees(30.0)
    val HOOD_ANGLE_TOLERANCE = Angle.fromDegrees(1.0)
    val INTAKE_DEPLOY_ANGLE_TOLERANCE = Angle.fromDegrees(1.0)
    val INTAKE_ROLLERS_RPM_TOLERANCE = AngularVelocity.fromRpm(20.0)
    val TURRET_ANGLE_TOLERANCE = Angle.fromDegrees(2.5)
    val INDEXER_RPM_TOLERANCE = AngularVelocity.fromRpm(20.0)
}

object SubsystemLimits {
    val INTAKE_ROLLERS_MAX_VELOCITY = AngularVelocity(0.0)..AngularVelocity(6000.0 / IntakeRollersConstants.Mechanical.GEAR_RATIO)
    val INTAKE_DEPLOY_ANGLE_LIMITS = Angle.fromDegrees(0.0)..Angle.fromDegrees(90.0)
    val FLYWHEEL_MAX_VELOCITY = AngularVelocity(0.0)..AngularVelocity(6000.0 / FlywheelConstants.Mechanical.GEAR_RATIO)
    val HOOD_ANGLE_LIMITS = Angle(0.0)..Angle.fromDegrees(90.0)
    val TURRET_ANGLE_LIMITS = Angle.fromDegrees(-180.0)..Angle.fromDegrees(180.0)
    val INDEXER_MAX_VELOCITY = AngularVelocity(0.0)..AngularVelocity(6000.0 / IndexerConstants.Mechanical.GEAR_RATIO)
    val FLYWHEEL_HARD_STOP_LIMITS = Angle(0.0)..Angle.fromDegrees(90.0)

}

object SubsystemPresetTargets {
    // Intake Rollers Preset RPM Targets //
    val INTAKE_ROLLERS_FLOOR_RPM = AngularVelocity.fromRpm(4000.0)
    val INTAKE_ROLLERS_FLOWER_RPM = AngularVelocity.fromRpm(4000.0)

    // Intake Deploy Preset Angle Targets //
    val INTAKE_DEPLOY_FLOOR_ANGLE = Angle.fromDegrees(0.0)
    val INTAKE_DEPLOY_FLOWER_ANGLE = Angle.fromDegrees(90.0)

    // Shooter Preset RPM Targets //
    val FLYWHEEL_PRESET_RPM = AngularVelocity.fromRpm(3000.0)

    // Hood Preset Angle Targets //
    val HOOD_PRESET_ANGLE = Angle.fromDegrees(90.0)
    val HOOD_HOME_ANGLE = Angle.fromDegrees(0.0)

    // Turret Preset Angle Targets //
    val TURRET_ZERO_ANGLE = Angle(0.0)

    // Indexer Preset RPM Targets //
    val INDEXER_PRESET_SHOOTING_RPM = AngularVelocity.fromRpm(4000.0)
    val INDEXER_PRESET_IDLE_RPM = AngularVelocity.fromRpm(2000.0)

    // Flywheel Hard Stop Preset Angle Targets //
    val FLYWHEEL_HARD_STOP_CLEARING_ANGLE = Angle(0.0)
    val FLYWHEEL_HARD_STOP_BLOCKING_ANGLE = Angle(90.0)
}

@Configurable
object SubsystemConfigurableTargets {
    // Intake Rollers Configurable RPM Targets //
    @JvmField
    var INTAKE_ROLLERS_CONFIGURABLE_RPM = 0.0

    // Intake Deploy Configurable Angle Targets //
    @JvmField
    var INTAKE_DEPLOY_CONFIGURABLE_DEGREES = 0.0

    // Shooter Configurable RPM Targets //
    @JvmField
    var FLYWHEEL_CONFIGURABLE_RPM = 0.0

    // Hood Configurable Angle Targets //
    @JvmField
    var HOOD_CONFIGURABLE_DEGREES = 0.0

    // Turret Configurable Angle Targets //
    @JvmField
    var TURRET_CONFIGURABLE_DEGREES = 0.0

    // Indexer Configurable RPM Targets //
    @JvmField
    var INDEXER_CONFIGURABLE_RPM = 0.0

    // Flywheel Hard Stop Configurable Angle Targets //
    @JvmField
    var FLYWHEEL_HARD_STOP_CONFIGURABLE_ANGLE = 0.0
}

@Configurable
object SubsystemControlGains {
    // Mecanum Angle PID Controller //
    @JvmField
    var MECANUM_ANGLE_PID = PIDCoefficients(0.1, 0.0, 0.0)

    // Intake PID and Feedforward Configurables //
    @JvmField
    var INTAKE_ROLLERS_MOTOR_PID = PIDCoefficients(0.1, 0.0, 0.0)
    @JvmField
    var INTAKE_ROLLERS_MOTOR_FEEDFORWARD = SimpleMotorFeedforward(0.0, 1.0, 0.0)

    // Intake Deploy PID and Feedforward Configurables //
    @JvmField
    var INTAKE_DEPLOY_SERVOS_PIDF = PIDFCoefficients(0.00575, 0.0, 0.0, 0.0)

    // Shooter PID and Feedforward Configurables //
    @JvmField
    var FLYWHEEL_MOTOR_PID = PIDCoefficients(0.01, 0.0, 0.0)
    @JvmField
    var FLYWHEEL_MOTOR_FEEDFORWARD = SimpleMotorFeedforward(0.0, 1.5, 0.0)

    // Turret PID and Feedforward Configurables //
    @JvmField
    var TURRET_SERVOS_PIDF = PIDFCoefficients(0.00575, 0.0, 0.0, 0.0)

    // Indexer PID and Feedforward Configurables //
    @JvmField
    var INDEXER_MOTOR_PID = PIDCoefficients(0.1, 0.0, 0.0)
    @JvmField
    var INDEXER_MOTOR_FEEDFORWARD = SimpleMotorFeedforward(0.0, 1.0, 0.0)
}