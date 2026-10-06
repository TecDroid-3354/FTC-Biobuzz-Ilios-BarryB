package org.firstinspires.ftc.teamcode.utils.units

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Plain-Kotlin geometry used by the localization package. Nothing in here depends on the FTC SDK,
 * Pedro or SolversLib, so the whole package can be unit-tested on a PC.
 *
 * Frame convention used everywhere in this package (unless a name says otherwise):
 *  - FIELD frame  : the standard FTC field frame (origin at field center). Heading is counter-clockwise positive.
 *  - ROBOT frame  : +x forward, +y left, origin at the robot's center (the point Pinpoint tracks).
 *  - Units        : inches and radians.
 * Convert to / from Pedro's coordinates ONLY at the boundary with [Pose.toPose2d] / [Pose2d.toPedroPose].
 */
data class Vec2(val x: Double, val y: Double) {
    operator fun plus(o: Vec2) = Vec2(x + o.x, y + o.y)
    operator fun minus(o: Vec2) = Vec2(x - o.x, y - o.y)
    operator fun times(k: Double) = Vec2(x * k, y * k)

    /** Rotates this vector counter-clockwise by [angleRad]. */
    fun rotated(angleRad: Double): Vec2 {
        val c = cos(angleRad)
        val s = sin(angleRad)
        return Vec2(c * x - s * y, s * x + c * y)
    }

    val norm: Double get() = hypot(x, y)
}

data class Vec3(val x: Double, val y: Double, val z: Double) {
    operator fun plus(o: Vec3) = Vec3(x + o.x, y + o.y, z + o.z)
    operator fun minus(o: Vec3) = Vec3(x - o.x, y - o.y, z - o.z)
    operator fun times(k: Double) = Vec3(x * k, y * k, z * k)
    fun dot(o: Vec3) = x * o.x + y * o.y + z * o.z
    fun cross(o: Vec3) = Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x)
    val norm: Double get() = sqrt(dot(this))
}

/** Wraps an angle to (-PI, PI]. */
fun wrapRadians(angle: Double): Double {
    var a = angle % (2.0 * PI)
    if (a > PI) a -= 2.0 * PI
    if (a <= -PI) a += 2.0 * PI
    return a
}

/**
 * A rigid hinge: a line in 3D space (a point on it plus a direction) that a body rotates about.
 * Used to compute where a tag mounted on a tipping structure ends up once it has rotated.
 * Rotation follows the right-hand rule about [axisDirection].
 */
class HingeJoint(private val pointOnAxis: Vec3, axisDirection: Vec3) {
    private val axis: Vec3 = axisDirection * (1.0 / axisDirection.norm)

    /** Rodrigues' rotation formula about the hinge line. */
    fun rotate(point: Vec3, angleRad: Double): Vec3 {
        val v = point - pointOnAxis
        val c = cos(angleRad)
        val s = sin(angleRad)
        val rotated = v * c + axis.cross(v) * s + axis * (axis.dot(v) * (1.0 - c))
        return pointOnAxis + rotated
    }
}

/** One shared clock (seconds) for odometry samples and vision capture times. */
fun nowSeconds(): Double = System.nanoTime() * 1e-9
