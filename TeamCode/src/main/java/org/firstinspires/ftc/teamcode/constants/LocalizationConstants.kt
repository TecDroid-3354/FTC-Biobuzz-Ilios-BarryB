package org.firstinspires.ftc.teamcode.constants

import org.firstinspires.ftc.teamcode.systems.TurretCameraMount
import org.firstinspires.ftc.teamcode.utils.localization.AprilTagMap
import org.firstinspires.ftc.teamcode.utils.localization.EstimatorConfig
import org.firstinspires.ftc.teamcode.utils.units.HingeJoint
import org.firstinspires.ftc.teamcode.utils.units.Vec3
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.FIELD_CENTER_PEDRO
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.HIVE_OFFSET_X
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.PIVOT_HEIGHT
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.REST_ANGLE
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightConstants
import org.firstinspires.ftc.teamcode.utils.extensions.toVec2

/**
 * Everything the pose estimator needs to know about the robot and the field.
 *
 * FRAME: the estimator runs in PEDRO coordinates (inches, origin at the field corner, heading counter-clockwise
 * from +x), exactly what `follower.pose()` returns, so there is no conversion inside it. The tag sheet was written in a
 * field-centered frame (+X toward Blue, +Y toward the back wall), which equals Pedro's frame shifted by FIELD_CENTER.
 * That equivalence assumes Red is on the audience's left (right-handed axes with Z up). TagMapCheck proves it on the bench.
 */
object LocalizationConstants {

    /** Measured from CAD. Accounts for turret and camera rotation not being on the robot's center. */
    val mount = TurretCameraMount(
        pivotInRobot = RobotTransformations.ROBOT_TO_TURRET_2D.toVec2(),
        cameraInTurret = RobotTransformations.TURRET_TO_LIMELIGHT_LENS.toVec2(),
        cameraYawOnTurret = RobotTransformations.TURRET_TO_LIMELIGHT_LENS.rotation.radians,
        cameraHeight = LimelightConstants.Mechanical.LIMELIGHT_LENS_HEIGHT.inches,
        cameraPitchUp = LimelightConstants.Mechanical.LIMELIGHT_PITCH_TO_HORIZONTAL.radians
    )

    const val TAG_HEIGHT_AT_LEVEL = 42.55  // tag center height with the hive level (never happens in a match)

    const val ARM = 14.27                    // tag cluster distance from the pivot along the arm

    // The hive turns about a horizontal line parallel to +x (the cells sit at +y and -y).
    private val redHinge = HingeJoint(
        Vec3(FIELD_CENTER_PEDRO.inches - HIVE_OFFSET_X.inches, FIELD_CENTER_PEDRO.inches, PIVOT_HEIGHT.inches),
        Vec3(1.0, 0.0, 0.0)
    )
    private val blueHinge = HingeJoint(
        Vec3(FIELD_CENTER_PEDRO.inches + HIVE_OFFSET_X.inches, FIELD_CENTER_PEDRO.inches, PIVOT_HEIGHT.inches),
        Vec3(1.0, 0.0, 0.0)
    )

    private fun AprilTagMap.hiveTag(id: Int, sheetX: Double, sheetY: Double, hinge: HingeJoint, group: String) {
        addBistable(id,
            Vec3(FIELD_CENTER_PEDRO.inches + sheetX, FIELD_CENTER_PEDRO.inches + sheetY, TAG_HEIGHT_AT_LEVEL), hinge, REST_ANGLE, group)
    }

    val biobuzzAprilTagDynamicMap = AprilTagMap().apply {
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