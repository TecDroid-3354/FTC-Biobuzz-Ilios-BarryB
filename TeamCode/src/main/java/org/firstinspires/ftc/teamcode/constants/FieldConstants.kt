package org.firstinspires.ftc.teamcode.constants

import com.pedropathing.math.Pose
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.skeletonarmy.marrow.zones.Point
import com.skeletonarmy.marrow.zones.PolygonZone
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.BLUE_GARDEN_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.BLUE_HIVE_CENTER_REFERENCE
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.BLUE_LOADING_ZONE_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.BOTTOM_FLOWER_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.FIELD_CENTER_PEDRO
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.FLOWER_WIDTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.GARDEN_DEPTH_Y
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.GARDEN_WIDTH_X
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.HIVE_FRAME_CENTER_IN_FIELD
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.autonomous.MirroredPoseFactory
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.HIVE_FRAME_WIDTH_Y
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.HIVE_FRAME_DEPTH_X
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.LEFT_FLOWER_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.LEFT_SIDE_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.LOADING_ZONE_DEPTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.LOADING_ZONE_WIDTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.RED_GARDEN_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.RED_HIVE_CENTER_REFERENCE
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.RED_LOADING_ZONE_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.RIGHT_FLOWER_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.RIGHT_SIDE_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.TOP_FLOWER_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.X_AXIS_WIDTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.Y_AXIS_HEIGHT
import org.firstinspires.ftc.teamcode.utils.extensions.toTranslation2d

object FieldDimensions {
    val X_AXIS_WIDTH = Distance.fromInches(141.5)
    val Y_AXIS_HEIGHT = Distance.fromInches(141.5)

    val FIELD_CENTER_PEDRO = Distance.fromInches(70.75) // Measure in x and y axis.

    // ---- HIVE ---- //
    val HIVE_FRAME_CENTER_IN_FIELD = Point(70.75, 70.75)
    val HIVE_FRAME_DEPTH_X = Distance.fromInches(38.95)
    val HIVE_FRAME_WIDTH_Y = Distance.fromInches(49.46)
    val HIVE_OFFSET_X = Distance.fromInches(12.75)          // red hive at -12.75, blue at +12.75
    val PIVOT_HEIGHT = Distance.fromInches(43.95)
    val REST_ANGLE = Math.toRadians(30.0)    // the hive rests at +30 or -30, nothing in between
    val RED_HIVE_CENTER_REFERENCE = Point(70.75, 58.0)
    val BLUE_HIVE_CENTER_REFERENCE = Point(70.75, 84.0)

    // ---- LOADING ZONES ---- //
    val RED_LOADING_ZONE_CENTER = Point(35.0, 5.0)
    val BLUE_LOADING_ZONE_CENTER = Point(106.0, 136.0)
    val LOADING_ZONE_WIDTH = Distance.fromInches(23.0)
    val LOADING_ZONE_DEPTH = Distance.fromInches(11.0)

    // ---- GARDEN ---- //

    val RED_GARDEN_CENTER = Point(140.0, 12.0)
    val BLUE_GARDEN_CENTER = Point(1.5, 130.0)
    val GARDEN_WIDTH_X = Distance.fromInches(1.0)
    val GARDEN_DEPTH_Y = Distance.fromInches(23.0)

    // ---- FIELD DIRECTION ZONES ---- //

    val LEFT_SIDE_CENTER = Point(35.0, 70.75)
    val RIGHT_SIDE_CENTER = Point(35.0, 70.75)

    // ---- FLOWER  ---- //

    val FLOWER_WIDTH = Distance.fromInches(2.4 + 1.89)
    val TOP_FLOWER_CENTER = Point(47.0, 139.0)
    val BOTTOM_FLOWER_CENTER = Point(94.0, 2.0)
    val RIGHT_FLOWER_CENTER = Point(139.0, 94.0)
    val LEFT_FLOWER_CENTER = Point(2.0, 47.0)
}

// TODO Confirm the veracity of these values.
object Zones {
    val HIVE_FRAME = PolygonZone(HIVE_FRAME_CENTER_IN_FIELD, HIVE_FRAME_WIDTH_Y.inches, HIVE_FRAME_DEPTH_X.inches)
    val HIVE_FRAME_TOP_BAR = PolygonZone(Point(FIELD_CENTER_PEDRO.inches, FIELD_CENTER_PEDRO.inches + HIVE_FRAME_WIDTH_Y.inches / 2.0), HIVE_FRAME_DEPTH_X.inches, 4.0)
    val HIVE_FRAME_BOTTOM_BAR = PolygonZone(Point(FIELD_CENTER_PEDRO.inches, FIELD_CENTER_PEDRO.inches - HIVE_FRAME_WIDTH_Y.inches / 2.0), HIVE_FRAME_DEPTH_X.inches, 4.0)
    val RED_LOADING_ZONE = PolygonZone(RED_LOADING_ZONE_CENTER, LOADING_ZONE_WIDTH.inches, LOADING_ZONE_DEPTH.inches)
    val BLUE_LOADING_ZONE = PolygonZone(BLUE_LOADING_ZONE_CENTER, LOADING_ZONE_WIDTH.inches, LOADING_ZONE_DEPTH.inches)
    val RED_GARDEN = PolygonZone(RED_GARDEN_CENTER, GARDEN_WIDTH_X.inches, GARDEN_DEPTH_Y.inches)
    val BLUE_GARDEN = PolygonZone(BLUE_GARDEN_CENTER, GARDEN_WIDTH_X.inches, GARDEN_DEPTH_Y.inches)
    val LEFT_SIDE = PolygonZone(LEFT_SIDE_CENTER, FIELD_CENTER_PEDRO.inches, Y_AXIS_HEIGHT.inches)
    val RIGHT_SIDE = PolygonZone(RIGHT_SIDE_CENTER, FIELD_CENTER_PEDRO.inches, Y_AXIS_HEIGHT.inches)
    val TOP_FLOWER = PolygonZone(TOP_FLOWER_CENTER, FLOWER_WIDTH.inches, FLOWER_WIDTH.inches)
    val BOTTOM_FLOWER = PolygonZone(BOTTOM_FLOWER_CENTER, FLOWER_WIDTH.inches, FLOWER_WIDTH.inches)
    val RIGHT_FLOWER = PolygonZone(RIGHT_FLOWER_CENTER, FLOWER_WIDTH.inches, FLOWER_WIDTH.inches)
    val LEFT_FLOWER = PolygonZone(LEFT_FLOWER_CENTER, FLOWER_WIDTH.inches, FLOWER_WIDTH.inches)
}

object ScoringTargets {
    // Red scoring targets
    val RED_HIVE_RIGHT_TOP_TARGET = Pose(80.0, 68.0)
    val RED_HIVE_RIGHT_BOTTOM_TARGET = Pose(80.0, 48.0)
    val RED_HIVE_LEFT_TOP_TARGET = Pose(61.0, 68.0)
    val RED_HIVE_LEFT_BOTTOM_TARGET = Pose(61.0, 48.0)

    // Blue scoring targets
    val BLUE_HIVE_RIGHT_TOP_TARGET = Pose(80.0, 95.0)
    val BLUE_HIVE_RIGHT_BOTTOM_TARGET = Pose(80.0, 75.0)
    val BLUE_HIVE_LEFT_TOP_TARGET = Pose(61.0, 95.0)
    val BLUE_HIVE_LEFT_BOTTOM_TARGET = Pose(61.0, 75.0)
}