package org.firstinspires.ftc.teamcode.utils.extensions

import com.pedropathing.math.Pose
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

/**
 * Per Pedro Pathing's documentation, transform from a standard [Pose2D] to Pedro's [Pose].
 * @return a new [Pose] in Pedro's Coordinate System.
 */
fun Pose2D.toPedroPose(): Pose {
    return Pose(
        this.y + 72.0,
        72.0 - this.x,
        this.h - Math.PI / 2
    )
}

fun Pose2d.toPedroPose(): Pose {
    return Pose2D(
        DistanceUnit.INCH,
        this.x, this.y,
        AngleUnit.RADIANS, this.heading).toPedroPose()
}

fun Pose.toPose2D(): Pose2D {
    return Pose2D(
        DistanceUnit.INCH,
        72.0 - this.y(), this.x() - 72.0,
        AngleUnit.RADIANS,
        this.heading() + Math.PI / 2
    )
}

fun Pose.toPose2d(): Pose2d {
    return Pose2d(
        this.toPose2D(),
        DistanceUnit.INCH,
        AngleUnit.RADIANS
    )
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