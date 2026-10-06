package org.firstinspires.ftc.teamcode.utils.localization

import org.firstinspires.ftc.teamcode.utils.units.HingeJoint
import org.firstinspires.ftc.teamcode.utils.units.Vec3

/**
 * Our own AprilTag layout, owned by the code (not by the Limelight's web UI).
 * Tag positions are the CENTER of each tag (x, y, height above floor), in inches, in the SAME frame as the odometry
 * pose you feed [PoseEstimator] (for this repo: Pedro's frame).
 *
 * A tag may have several candidate placements. A fixed tag has exactly one. A tag on a structure that
 * rests in one of two positions (the HIVE) has two. [PoseEstimator] decides which candidate explains each measurement.
 */
class AprilTagMap {
    private class Entry(val placements: List<Vec3>, val group: String?, val sigma: Double)

    private val entries = HashMap<Int, Entry>()

    /** A tag that never moves. [sigma] is how well you know its position (inches). */
    fun addFixed(id: Int, center: Vec3, sigma: Double = 0.0): AprilTagMap {
        entries[id] = Entry(listOf(center), null, sigma)
        return this
    }

    /**
     * A tag rigidly attached to a structure that rotates about [hinge] and rests at either +[restAngleRad]
     * or -[restAngleRad] (never in the middle). [nominal] is the tag's position with the structure level (angle 0).
     * Candidate 0 is rotated by +restAngle, candidate 1 by -restAngle (right-hand rule about the hinge direction).
     *
     * All tags that share a [group] belong to the same structure, so they share one state; the estimator
     * tracks that state across frames instead of guessing it from scratch for every tag.
     * [placementSigma] covers how far the real resting angle may be from the nominal one (inches).
     */
    fun addBistable(
        id: Int, nominal: Vec3, hinge: HingeJoint, restAngleRad: Double, group: String, placementSigma: Double = 1.0
    ): AprilTagMap {
        entries[id] = Entry(listOf(hinge.rotate(nominal, restAngleRad), hinge.rotate(nominal, -restAngleRad)), group, placementSigma)
        return this
    }

    fun candidates(id: Int): List<Vec3> = entries[id]?.placements ?: emptyList()
    fun group(id: Int): String? = entries[id]?.group
    fun placementSigma(id: Int): Double = entries[id]?.sigma ?: 0.0
}
