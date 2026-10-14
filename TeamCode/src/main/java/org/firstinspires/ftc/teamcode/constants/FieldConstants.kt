package org.firstinspires.ftc.teamcode.constants

import com.pedropathing.math.Pose
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.skeletonarmy.marrow.zones.Point
import com.skeletonarmy.marrow.zones.PolygonZone
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.BLUE_GARDEN_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.BLUE_LOADING_ZONE_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.FIELD_CENTER_PEDRO
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.GARDEN_DEPTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.GARDEN_WIDTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.HIVE_CENTER_IN_FIELD
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.autonomous.MirroredPoseFactory
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.HIVE_FRAME_WIDTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.HIVE_FRAME_DEPTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.LEFT_SIDE_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.LOADING_ZONE_DEPTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.LOADING_ZONE_WIDTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.RED_GARDEN_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.RED_LOADING_ZONE_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.RIGHT_SIDE_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.Y_AXIS_HEIGHT

object FieldDimensions {
    val X_AXIS_WIDTH = Distance.fromInches(141.5)
    val Y_AXIS_HEIGHT = Distance.fromInches(141.5)

    val FIELD_CENTER_PEDRO = Distance.fromInches(70.75) // Measure in x and y axis.

    // ---- HIVE ---- //
    val HIVE_CENTER_IN_FIELD = Point(70.75, 70.75)
    val HIVE_FRAME_DEPTH = Distance.fromInches(38.95)
    val HIVE_FRAME_WIDTH = Distance.fromInches(49.46)
    val HIVE_OFFSET_X = Distance.fromInches(12.75)          // red hive at -12.75, blue at +12.75
    val PIVOT_HEIGHT = Distance.fromInches(43.95)
    val REST_ANGLE = Math.toRadians(30.0)    // the hive rests at +30 or -30, nothing in between

    // ---- LOADING ZONES ---- //
    val RED_LOADING_ZONE_CENTER = Point(35.0, 5.0)
    val BLUE_LOADING_ZONE_CENTER = Point(106.0, 136.0)
    val LOADING_ZONE_WIDTH = Distance.fromInches(23.0)
    val LOADING_ZONE_DEPTH = Distance.fromInches(11.0)

    // ---- GARDEN ---- //

    val RED_GARDEN_CENTER = Point(140.0, 12.0)
    val BLUE_GARDEN_CENTER = Point(1.5, 130.0)
    val GARDEN_WIDTH = Distance.fromInches(1.0)
    val GARDEN_DEPTH = Distance.fromInches(23.0)

    // ---- FIELD DIRECTION ZONES ---- //

    val LEFT_SIDE_CENTER = Point(35.0, 70.75)
    val RIGHT_SIDE_CENTER = Point(35.0, 70.75)

}

object Zones {
    val HIVE_FRAME = PolygonZone(HIVE_CENTER_IN_FIELD, HIVE_FRAME_WIDTH.inches, HIVE_FRAME_DEPTH.inches)
    val HIVE_TOP_BAR = PolygonZone(Point(70.75.minus(HIVE_FRAME_DEPTH.inches / 2), 70.75.plus(HIVE_FRAME_WIDTH.inches / 2)), Point(70.75.plus(HIVE_FRAME_DEPTH.inches / 2), 70.75.plus(HIVE_FRAME_WIDTH.inches / 2)), 4.0)
    val HIVE_BOTTOM_BAR = PolygonZone(Point(70.75.minus(HIVE_FRAME_DEPTH.inches / 2), 70.75.minus(HIVE_FRAME_WIDTH.inches / 2)), Point(70.75.plus(HIVE_FRAME_DEPTH.inches / 2), 70.75.minus(HIVE_FRAME_WIDTH.inches / 2)), 4.0)
    val RED_LOADING_ZONE = PolygonZone(RED_LOADING_ZONE_CENTER, LOADING_ZONE_WIDTH.inches, LOADING_ZONE_DEPTH.inches)
    val BLUE_LOADING_ZONE = PolygonZone(BLUE_LOADING_ZONE_CENTER, LOADING_ZONE_WIDTH.inches, LOADING_ZONE_DEPTH.inches)
    val RED_GARDEN = PolygonZone(RED_GARDEN_CENTER, GARDEN_WIDTH.inches, GARDEN_DEPTH.inches)
    val BLUE_GARDEN = PolygonZone(BLUE_GARDEN_CENTER, GARDEN_WIDTH.inches, GARDEN_DEPTH.inches)
    val LEFT_SIDE = PolygonZone(LEFT_SIDE_CENTER, FIELD_CENTER_PEDRO.inches, Y_AXIS_HEIGHT.inches)
    val RIGHT_SIDE = PolygonZone(RIGHT_SIDE_CENTER, FIELD_CENTER_PEDRO.inches, Y_AXIS_HEIGHT.inches)
}

/**
 * Creates two [Pose]s in the field, one for the right and one for the left HIVE.
 * Implements [MirroredPoseFactory] and creates a [com.pedropathing.api.PoseFactory] and mirrors
 * the poses depending on the alliance. The [Pose] is in Pedro Pathing Coordinates. When turned into a [Translation2d]
 * it converts it to standard FTC Coordinates first
 */
class ScoringTargets(alliance: Alliance): MirroredPoseFactory(AngleUnit.DEGREES, alliance) {

    val rightHivePose: Pose = poseFactory.of(0.0, 0.0, 0.0)
    val leftHivePose: Pose = poseFactory.of(0.0, 0.0, 0.0)
}