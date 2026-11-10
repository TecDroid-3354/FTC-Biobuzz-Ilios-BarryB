package org.firstinspires.ftc.teamcode.constants

import com.pedropathing.math.Pose
import com.skeletonarmy.marrow.zones.Point
import com.skeletonarmy.marrow.zones.PolygonZone
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.BLUE_GARDEN_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.BLUE_LOADING_ZONE_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.BOTTOM_FLOWER_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.FIELD_CENTER_PEDRO
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.FLOWER_WIDTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.GARDEN_DEPTH_Y
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.GARDEN_WIDTH_X
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.HIVE_FRAME_CENTER_IN_FIELD
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.HIVE_FRAME_WIDTH_X
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.HIVE_FRAME_DEPTH_Y
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.LEFT_FLOWER_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.TOP_SIDE_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.LOADING_ZONE_DEPTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.LOADING_ZONE_WIDTH
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.RED_GARDEN_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.RED_LOADING_ZONE_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.RIGHT_FLOWER_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.BOTTOM_SIDE_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.TOP_FLOWER_CENTER
import org.firstinspires.ftc.teamcode.constants.FieldDimensions.X_AXIS_WIDTH

object FieldDimensions {
    val X_AXIS_WIDTH                    = Distance.fromInches(141.5)
    val Y_AXIS_HEIGHT                   = Distance.fromInches(141.5)

    val FIELD_CENTER_PEDRO              = Distance.fromInches(70.75) // Measure in x and y axis.

    // ---- HIVE ---- //
    // TRUE
    val HIVE_FRAME_CENTER_IN_FIELD      = Point(70.75, 70.75)
    val HIVE_FRAME_DEPTH_Y              = Distance.fromInches(38.95)
    val HIVE_FRAME_WIDTH_X              = Distance.fromInches(49.46)
    val HIVE_OFFSET_X                   = Distance.fromInches(12.75)// red hive at -12.75, blue at +12.75
    val PIVOT_HEIGHT                    = Distance.fromInches(43.95)
    val REST_ANGLE                      = Math.toRadians(30.0)// the hive rests at +30 or -30, nothing in between
    val RED_HIVE_CENTER_REFERENCE       = Point(58.0, 70.75)
    val BLUE_HIVE_CENTER_REFERENCE      = Point(84.0, 70.75)

    // ---- LOADING ZONES ---- //
    // TRUE
    val RED_LOADING_ZONE_CENTER         = Point(4.0, 106.0)
    val BLUE_LOADING_ZONE_CENTER        = Point(106.0, 136.0)
    val LOADING_ZONE_WIDTH              = Distance.fromInches(23.0)
    val LOADING_ZONE_DEPTH              = Distance.fromInches(11.0)

    // ---- GARDEN ---- //
    // TRUE

    val RED_GARDEN_CENTER               = Point(12.0, 1.5)
    val BLUE_GARDEN_CENTER              = Point(130.0, 142.0)
    val GARDEN_DEPTH_Y                  = Distance.fromInches(1.0)
    val GARDEN_WIDTH_X                  = Distance.fromInches(23.0)

    // ---- FIELD DIRECTION ZONES ---- //
    // TRUE

    val TOP_SIDE_CENTER                = Point(75.0, 105.75)
    val BOTTOM_SIDE_CENTER               = Point(75.0, 35.75)

    // ---- FLOWER  ---- //
    // TRUE

    val FLOWER_WIDTH                    = Distance.fromInches(2.4 + 1.89)
    val TOP_FLOWER_CENTER               = Point(47.0, 139.0)
    val BOTTOM_FLOWER_CENTER            = Point(94.0, 2.0)
    val RIGHT_FLOWER_CENTER             = Point(139.0, 94.0)
    val LEFT_FLOWER_CENTER              = Point(2.0, 47.0)
}

// Frame changed, positions are now validated.
object Zones {
    val HIVE_FRAME      = PolygonZone(HIVE_FRAME_CENTER_IN_FIELD, HIVE_FRAME_WIDTH_X.inches, HIVE_FRAME_DEPTH_Y.inches)
    val HIVE_FRAME_RIGHT_BAR = PolygonZone(Point(FIELD_CENTER_PEDRO.inches + HIVE_FRAME_WIDTH_X.inches / 2.0, FIELD_CENTER_PEDRO.inches), 4.0, HIVE_FRAME_DEPTH_Y.inches)
    val HIVE_FRAME_LEFT_BAR = PolygonZone(Point(FIELD_CENTER_PEDRO.inches  - HIVE_FRAME_WIDTH_X.inches / 2.0, FIELD_CENTER_PEDRO.inches), 4.0, HIVE_FRAME_DEPTH_Y.inches)
    val RED_LOADING_ZONE = PolygonZone(RED_LOADING_ZONE_CENTER, LOADING_ZONE_DEPTH.inches, LOADING_ZONE_WIDTH.inches)
    val BLUE_LOADING_ZONE = PolygonZone(BLUE_LOADING_ZONE_CENTER, LOADING_ZONE_DEPTH.inches, LOADING_ZONE_WIDTH.inches)
    val RED_GARDEN      = PolygonZone(RED_GARDEN_CENTER, GARDEN_WIDTH_X.inches, GARDEN_DEPTH_Y.inches)
    val BLUE_GARDEN     = PolygonZone(BLUE_GARDEN_CENTER, GARDEN_WIDTH_X.inches, GARDEN_DEPTH_Y.inches)
    val TOP_SIDE        = PolygonZone(TOP_SIDE_CENTER, X_AXIS_WIDTH.inches, FIELD_CENTER_PEDRO.inches)
    val BOTTOM_SIDE      = PolygonZone(BOTTOM_SIDE_CENTER, X_AXIS_WIDTH.inches, FIELD_CENTER_PEDRO.inches)
    // plus 2 inches is an arbitrary value which ensures the obstacle is cleared with ease.
    val TOP_FLOWER      = PolygonZone(TOP_FLOWER_CENTER, FLOWER_WIDTH.inches + 2.0, FLOWER_WIDTH.inches + 2.0)
    val BOTTOM_FLOWER   = PolygonZone(BOTTOM_FLOWER_CENTER, FLOWER_WIDTH.inches + 2.0, FLOWER_WIDTH.inches + 2.0)
    val RIGHT_FLOWER    = PolygonZone(RIGHT_FLOWER_CENTER, FLOWER_WIDTH.inches + 2.0, FLOWER_WIDTH.inches + 2.0)
    val LEFT_FLOWER     = PolygonZone(LEFT_FLOWER_CENTER, FLOWER_WIDTH.inches + 2.0, FLOWER_WIDTH.inches + 2.0)
}

object ScoringTargets {
    // Red scoring targets
    val RED_HIVE_TOP_RIGHT_TARGET       = Pose(64.0, 82.0)
    val RED_HIVE_TOP_LEFT_TARGET        = Pose(52.0, 82.0)
    val RED_HIVE_BOTTOM_RIGHT_TARGET    = Pose(64.0, 58.0)
    val RED_HIVE_BOTTOM_LEFT_TARGET     = Pose(52.0, 58.0)

    // Blue scoring targets
    val BLUE_HIVE_TOP_RIGHT_TARGET      = Pose(90.0, 82.0)
    val BLUE_HIVE_TOP_LEFT_TARGET       = Pose(78.0, 82.0)
    val BLUE_HIVE_BOTTOM_RIGHT_TARGET   = Pose(90.0, 58.0)
    val BLUE_HIVE_BOTTOM_LEFT_TARGET    = Pose(78.0, 58.0)
}

object FieldTargets {
    val RIGHT_FLOWER_INTAKE_POSITION    = Pose(124.0, 94.0, 0.0)
    val TOP_FLOWER_INTAKE_POSITION      = Pose(47.0, 124.0, Math.PI / 2)
    val LEFT_FLOWER_INTAKE_POSITION     = Pose(17.0, 47.0, Math.PI)
    val BOTTOM_FLOWER_INTAKE_POSITION   = Pose(94.0, 19.0, -Math.PI / 2)

    val FLOWER_LIST                     = listOf(RIGHT_FLOWER_INTAKE_POSITION, TOP_FLOWER_INTAKE_POSITION, LEFT_FLOWER_INTAKE_POSITION, BOTTOM_FLOWER_INTAKE_POSITION)

    val BLUE_PARK_LEFT                  = Pose(91.0, 124.0, Math.PI / 2)
    val BLUE_PARK_RIGHT                 = Pose(121.0, 124.0, Math.PI / 2)
    val RED_PARK_LEFT                   = Pose(20.0, 14.0, 3 * Math.PI / 2)
    val RED_PARK_RIGHT                  = Pose(52.0, 14.0, 3 * Math.PI / 2)
}