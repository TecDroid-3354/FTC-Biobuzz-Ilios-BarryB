package org.firstinspires.ftc.teamcode.constants

import org.firstinspires.ftc.teamcode.systems.TurretCameraMount
import org.firstinspires.ftc.teamcode.utils.localization.AprilTagMap
import org.firstinspires.ftc.teamcode.utils.localization.EstimatorConfig
import org.firstinspires.ftc.teamcode.utils.units.HingeJoint
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import org.firstinspires.ftc.teamcode.utils.units.Vec3

/**
 * Everything the pose estimator needs to know about the robot and the field.
 *
 * FRAME: the estimator runs in PEDRO coordinates (inches, origin at the field corner, heading counter-clockwise
 * from +x), exactly what `follower.pose()` returns, so there is no conversion inside it. The tag sheet was written in a
 * field-centered frame (+X toward Blue, +Y toward the back wall), which equals Pedro's frame shifted by FIELD_CENTER.
 * That equivalence assumes Red is on the audience's left (right-handed axes with Z up). TagMapCheck proves it on the bench.
 */
object LocalizationConstants {

    /** Measure these from CAD. They are the whole "turret is not at the robot's center" fix. */
    val mount = TurretCameraMount(
        pivotInRobot = Vec2(
            0.0,
            0.0
        ),          // TODO turret axis relative to the robot center (+x forward, +y left)
        cameraInTurret = Vec2(
            0.0,
            0.0
        ),        // TODO lens position relative to the turret axis, turret facing +x
        cameraYawOnTurret = 0.0,                // TODO 0 if the lens looks where the turret looks
        cameraHeight = 0.0,                     // TODO lens height above the floor (must be > 0)
        cameraPitchUp = Math.toRadians(0.0)     // TODO lens tilt above horizontal. Every tag is above the camera, so this is > 0
    )

    // ---- BIOBUZZ HIVE, from the team's tag sheet (field-centered inches; added to FIELD_CENTER below) ----
    private const val FIELD_CENTER = 72.0
    private const val HIVE_OFFSET_X = 12.75          // red hive at -12.75, blue at +12.75
    private const val PIVOT_HEIGHT = 43.95
    private const val TAG_HEIGHT_AT_LEVEL = 36.95    // tag center height with the hive level (never happens in a match)
    private const val ARM = 15.44                    // tag cluster distance from the pivot along the arm
    private val REST_ANGLE = Math.toRadians(30.0)    // the hive rests at +30 or -30, nothing in between

    // The hive turns about a horizontal line parallel to +x (the cells sit at +y and -y).
    private val redHinge = HingeJoint(
        Vec3(FIELD_CENTER - HIVE_OFFSET_X, FIELD_CENTER, PIVOT_HEIGHT),
        Vec3(1.0, 0.0, 0.0)
    )
    private val blueHinge = HingeJoint(
        Vec3(FIELD_CENTER + HIVE_OFFSET_X, FIELD_CENTER, PIVOT_HEIGHT),
        Vec3(1.0, 0.0, 0.0)
    )

    private fun AprilTagMap.hiveTag(id: Int, sheetX: Double, sheetY: Double, hinge: HingeJoint, group: String) {
        addBistable(id,
            Vec3(FIELD_CENTER + sheetX, FIELD_CENTER + sheetY, TAG_HEIGHT_AT_LEVEL), hinge, REST_ANGLE, group)
    }

    val tagMap = AprilTagMap().apply {
        // Red hive, scoring side (+Y)
        hiveTag(30, -6.25, ARM, redHinge, "red");  hiveTag(31, -10.00, ARM, redHinge, "red")
        hiveTag(32, -15.50, ARM, redHinge, "red"); hiveTag(33, -19.25, ARM, redHinge, "red")
        // Red hive, audience side (-Y)
        hiveTag(34, -19.25, -ARM, redHinge, "red"); hiveTag(35, -15.50, -ARM, redHinge, "red")
        hiveTag(36, -10.00, -ARM, redHinge, "red"); hiveTag(37, -6.25, -ARM, redHinge, "red")
        // Blue hive, audience side (-Y)
        hiveTag(38, 6.25, -ARM, blueHinge, "blue");  hiveTag(39, 10.00, -ARM, blueHinge, "blue")
        hiveTag(40, 15.50, -ARM, blueHinge, "blue"); hiveTag(41, 19.25, -ARM, blueHinge, "blue")
        // Blue hive, scoring side (+Y)
        hiveTag(42, 19.25, ARM, blueHinge, "blue"); hiveTag(43, 15.50, ARM, blueHinge, "blue")
        hiveTag(44, 10.00, ARM, blueHinge, "blue"); hiveTag(45, 6.25, ARM, blueHinge, "blue")
    }

    /** Starting values are fine for the first test; tune from logged data. */
    val estimatorConfig = EstimatorConfig()
}