package org.firstinspires.ftc.teamcode.utils.extensions

import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.skeletonarmy.marrow.weaver.PathResult
import com.skeletonarmy.marrow.weaver.PedroPathingConverter
import com.skeletonarmy.marrow.zones.Point
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

/**
 * Per Pedro Pathing's documentation, transform from a standard [Pose2D] to Pedro's [Pose].
 * @return a new [Pose] in Pedro's Coordinate System.
 */
fun Pose2D.toPedroCoordinates(): Pose {
    return Pose(
        this.y + 70.75,
        70.75 - this.x,
        this.h - Math.PI / 2
    )
}

fun Pose2d.toPedroCoordinates(): Pose {
    return Pose2D(
        DistanceUnit.INCH,
        this.x, this.y,
        AngleUnit.RADIANS, this.heading).toPedroCoordinates()
}

fun Pose.toFTCCoordinates(): Pose2D {
    return Pose2D(
        DistanceUnit.INCH,
        70.75 - this.y(), this.x() - 70.75,
        AngleUnit.RADIANS,
        this.heading() + Math.PI / 2
    )
}

fun Pose.toPose2d(): Pose2d {
    return Pose2d(
        this.x(),
        this.y(),
        this.heading()
    )
}

fun Pose.toPoint(): Point {
    return PedroPathingConverter.toPoint(this)
}

fun PathResult.toPedroPath(): Path {
    return PedroPathingConverter.toPath(this)
}

fun Pose.toTranslation2d(): Translation2d {
    return this.toPose2d().translation
}

/**
 * Gets the x component of the [Pose2D] in inches.
 */
val Pose2D.x : Double; get() = this.getX(DistanceUnit.INCH)
/**
 * Gets the y component of the [Pose2D] in inches.
 */
val Pose2D.y : Double; get() = this.getY(DistanceUnit.INCH)

/**
 * Gets the rotation component of the [Pose2D] in radians
 */
val Pose2D.h : Double ; get() = this.getHeading(AngleUnit.RADIANS)