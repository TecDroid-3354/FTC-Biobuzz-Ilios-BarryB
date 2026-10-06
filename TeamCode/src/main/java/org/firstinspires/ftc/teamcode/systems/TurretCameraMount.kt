package org.firstinspires.ftc.teamcode.systems

import org.firstinspires.ftc.teamcode.utils.units.Vec2
import org.firstinspires.ftc.teamcode.utils.units.Vec3
import org.firstinspires.ftc.teamcode.utils.units.wrapRadians
import kotlin.math.cos
import kotlin.math.sin

/**
 * Describes a camera riding on a turret whose axis is NOT at the robot's center.
 *
 * Turret angle convention (matches [org.firstinspires.ftc.teamcode.subsystems.turret.Turret.calculateTurretAngle]):
 * turretAngle = 0 means the turret faces the robot's forward direction, positive is counter-clockwise.
 * Therefore, the turret's heading in the FIELD frame is `robotHeading + turretAngle`.
 *
 * @param pivotInRobot       turret axis position in the ROBOT frame (inches).
 * @param cameraInTurret     camera lens position relative to the turret axis, in the TURRET frame at turretAngle = 0 (inches).
 *                           +x is the direction the turret faces.
 * @param cameraYawOnTurret  camera's yaw relative to the turret's facing direction (rad). 0 if it looks the same way the turret does.
 * @param cameraHeight       lens height above the floor (inches).
 * @param cameraPitchUp      camera tilt above horizontal (rad, positive = lens pointing up).
 */
class TurretCameraMount(
    val pivotInRobot: Vec2,
    val cameraInTurret: Vec2,
    val cameraYawOnTurret: Double,
    val cameraHeight: Double,
    val cameraPitchUp: Double
) {
    init {
        require(cameraHeight > 0.0) { "Measure the Limelight's lens height above the floor and set it in LocalizationConstants.mount" }
    }

    /** Camera position relative to the robot's center, in the ROBOT frame, for a given turret angle. */
    fun cameraOffsetInRobot(turretAngle: Double): Vec2 =
        pivotInRobot + cameraInTurret.rotated(turretAngle)

    /** Direction the camera looks at in the FIELD frame. */
    fun cameraYawInField(robotHeading: Double, turretAngle: Double): Double =
        wrapRadians(robotHeading + turretAngle + cameraYawOnTurret)

    /**
     * Turns a vector expressed in the Limelight camera frame (x right, y down, z out of the lens)
     * into a gravity-aligned frame that rotates only with the camera's yaw:
     * x = forward, y = left, z = up. The camera's pitch is removed here.
     */
    fun levelize(right: Double, down: Double, out: Double): Vec3 {
        val cp = cos(cameraPitchUp)
        val sp = sin(cameraPitchUp)
        return Vec3(
            x = out * cp + down * sp,
            y = -right,
            z = out * sp - down * cp
        )
    }

    /**
     * Robot-center position in the FIELD frame, given where the camera is in the field frame.
     * This is the "pivot is not at the robot center" correction.
     */
    fun robotPositionFromCamera(cameraField: Vec2, robotHeading: Double, turretAngle: Double): Vec2 =
        cameraField - cameraOffsetInRobot(turretAngle).rotated(robotHeading)
}