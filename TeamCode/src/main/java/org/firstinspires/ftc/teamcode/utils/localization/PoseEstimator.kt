package org.firstinspires.ftc.teamcode.utils.localization

import org.firstinspires.ftc.teamcode.systems.TurretCameraMount
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import org.firstinspires.ftc.teamcode.utils.units.Vec3
import org.firstinspires.ftc.teamcode.utils.units.wrapRadians
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/** One AprilTag as the Limelight reports it: the tag center in CAMERA space (x right, y down, z out), inches. */
class TagObservation(val id: Int, val right: Double, val down: Double, val out: Double)

/** Everything the Limelight saw in a single camera frame. [captureTimeSeconds] is on the robot's clock. */
class VisionFrame(val captureTimeSeconds: Double, val tags: List<TagObservation>)

/** All the tunables in one place. Defaults are sensible starting points, tune them from logged data. */
data class EstimatorConfig(
    val historySeconds: Double = 1.5,
    val initialVariance: Double = 4.0,            // in^2, uncertainty of the starting pose

    // Process noise: how fast we stop trusting odometry
    val odometryVariancePerInch: Double = 0.004,  // in^2 per inch driven
    val odometryVariancePerRadian: Double = 0.3,  // in^2 per radian turned (wheel-offset pods slip when spinning)
    val odometryVariancePerSecond: Double = 0.0004,

    // Measurement noise model
    val rangeSigmaPerInchSq: Double = 0.0004,     // sigma_range = k * range^2 (3.25 in tags: error grows as range^2 / tag size)
    val rangeSigmaFloor: Double = 0.5,            // in
    val pixelBearingSigma: Double = Math.toRadians(0.5),
    val turretAngleSigma: Double = Math.toRadians(0.3),
    val headingSigma: Double = Math.toRadians(0.5),
    val captureTimingSigma: Double = 0.010,       // s, how well we know when the picture was taken
    val heightSigmaBase: Double = 1.0,            // in
    val heightSigmaPerInch: Double = 0.015,       // in of height error per inch of range (pitch error)

    // Acceptance
    val minRange: Double = 8.0,
    val maxRange: Double = 100.0,                 // a 3.25 in tag is only ~20 px wide at 100 in
    val gateChiSq: Double = 11.34,                // 3 DoF (x, y, height), 99%
    val ambiguityMargin: Double = 4.0,            // best hypothesis must beat the runner-up by this much
    val rejectsBeforeRecovery: Int = 10,
    val recoveryVariance: Double = 25.0,

    // Structure state tracking (tags that share a group)
    val useGroupBelief: Boolean = true,
    val beliefDecay: Double = 0.97,               // forgets old evidence, so a structure that tips is noticed
    val beliefLimit: Double = 12.0,               // caps how sure we can be (log-likelihood ratio)
    val beliefStepLimit: Double = 4.0             // caps how much one tag can move the belief
)

class EstimatorStats {
    var acceptedTags = 0; internal set
    var gatedTags = 0; internal set
    var ambiguousTags = 0; internal set
    var unmappedTags = 0; internal set
    var outOfRangeTags = 0; internal set
    var lastMahalanobisSq = 0.0; internal set
    var lastHypothesisIndex = -1; internal set
}

/** Result of [PoseEstimator.diagnose]: what one tag says the robot position is, under each possible tag placement. */
class TagDiagnostic(
    val id: Int, val candidateIndex: Int, val robotX: Double, val robotY: Double,
    val errorInches: Double, val heightResidualInches: Double
)

/**
 * Fuses Pinpoint odometry with Limelight AprilTag detections for a camera mounted on a turret.
 *
 * Per frame the pipeline is:
 *  1. look up robot heading and turret angle AT THE TIME THE PICTURE WAS TAKEN
 *  2. camera yaw in the field = heading + turretAngle (+ mount yaw)
 *  3. camera position = tag position - R(cameraYaw) * (tag position relative to the camera, leveled)
 *  4. robot center = camera position - R(heading) * (pivot + R(turretAngle) * cameraOffset)
 *  5. for tags with several possible placements (the HIVE rests at +/- an angle) test every candidate against the
 *     filter AND the camera's known height, and keep a running belief per structure so all its tags agree;
 *     reject the tag if the state is still ambiguous
 *  6. fuse with a Kalman update whose noise depends on range, bearing and how fast the camera is turning
 *
 * Output position = odometry position + accumulated vision correction. The raw odometry is never
 * overwritten, so it can always be logged and compared.
 */
class PoseEstimator(
    private val mount: TurretCameraMount,
    private val tagMap: AprilTagMap,
    private val config: EstimatorConfig = EstimatorConfig()
) {
    private val filter = PositionKalmanFilter(config.historySeconds)
    val stats = EstimatorStats()

    private class Sample(val t: Double, val heading: Double, val turret: Double)
    private val samples = ArrayList<Sample>()

    private var hasOdometry = false
    private var lastX = 0.0
    private var lastY = 0.0
    private var lastHeading = 0.0
    private var lastT = 0.0
    private var consecutiveGated = 0
    private val beliefs = HashMap<String, Double>()

    /** Positive: the structure is believed to be at its +rest angle (candidate 0). Negative: -rest angle. 0 = unknown. */
    fun groupBelief(group: String): Double = beliefs[group] ?: 0.0

    val x: Double get() = filter.x
    val y: Double get() = filter.y
    val isInitialized: Boolean get() = filter.isInitialized

    /** Call whenever the follower's pose is set by hand (start of an OpMode, manual relocalization). */
    fun resetPose() {
        hasOdometry = false
        samples.clear()
        consecutiveGated = 0
    }

    /** Call once per loop with the follower's odometry pose (FIELD frame) and the MEASURED turret angle. */
    fun updateOdometry(t: Double, odoX: Double, odoY: Double, odoHeading: Double, turretAngle: Double) {
        if (!hasOdometry) {
            filter.reset(t, odoX, odoY, config.initialVariance)
            hasOdometry = true
        } else {
            val dx = odoX - lastX
            val dy = odoY - lastY
            val q = config.odometryVariancePerInch * hypot(dx, dy) +
                    config.odometryVariancePerRadian * abs(wrapRadians(odoHeading - lastHeading)) +
                    config.odometryVariancePerSecond * (t - lastT)
            filter.predict(t, dx, dy, q)
        }
        samples.add(Sample(t, odoHeading, turretAngle))
        while (samples.size > 2 && samples[1].t < t - config.historySeconds) samples.removeAt(0)
        lastX = odoX; lastY = odoY; lastHeading = odoHeading; lastT = t
    }

    private fun headingAt(t: Double): Pair<Double, Double>? {
        if (samples.isEmpty() || t < samples.first().t) return null
        if (t >= samples.last().t) return samples.last().let { Pair(it.heading, it.turret) }
        var i = 0
        while (i + 1 < samples.size && samples[i + 1].t <= t) i++
        val a = samples[i]
        val b = samples[i + 1]
        val k = (t - a.t) / (b.t - a.t)
        return Pair(a.heading + k * wrapRadians(b.heading - a.heading), a.turret + k * (b.turret - a.turret))
    }

    private fun cameraYawRate(t: Double): Double {
        val a = headingAt(t - 0.03) ?: return 0.0
        val b = headingAt(t + 0.03) ?: return 0.0
        val ya = mount.cameraYawInField(a.first, a.second)
        val yb = mount.cameraYawInField(b.first, b.second)
        return wrapRadians(yb - ya) / 0.06
    }

    private class Candidate(val index: Int, val robot: Vec2, val score: Double, val mahalanobisSq: Double)

    private class Solved(val robot: Vec2, val impliedCameraHeight: Double)

    private fun solve(tag: TagObservation, placement: Vec3, heading: Double, turret: Double): Solved {
        val leveled = mount.levelize(tag.right, tag.down, tag.out)
        val toTagField = Vec2(leveled.x, leveled.y).rotated(mount.cameraYawInField(heading, turret))
        val cameraField = Vec2(placement.x, placement.y) - toTagField
        return Solved(mount.robotPositionFromCamera(cameraField, heading, turret), placement.z - leveled.z)
    }

    /**
     * Bench check. With the robot standing at a position you measured by hand, shows what every tag implies under
     * every candidate placement. The right candidate should land within a couple of inches and ~0 height residual.
     * A wrong tag layout, wrong axes, wrong tag size or wrong mount numbers show up here as errors of tens of inches.
     */
    fun diagnose(
        tags: List<TagObservation>, knownX: Double, knownY: Double, heading: Double, turret: Double
    ): List<TagDiagnostic> {
        val out = ArrayList<TagDiagnostic>()
        for (tag in tags) {
            for ((index, placement) in tagMap.candidates(tag.id).withIndex()) {
                val r = solve(tag, placement, heading, turret)
                out.add(TagDiagnostic(tag.id, index, r.robot.x, r.robot.y,
                    hypot(r.robot.x - knownX, r.robot.y - knownY), r.impliedCameraHeight - mount.cameraHeight))
            }
        }
        return out
    }

    /** Fuses every usable tag in [frame]. Returns how many tags were accepted. */
    fun addVisionFrame(frame: VisionFrame): Int {
        if (!filter.isInitialized) return 0
        val t = frame.captureTimeSeconds
        val (heading, turret) = headingAt(t) ?: return 0

        val usable = frame.tags.filter { tag ->
            val range = hypot(tag.out, tag.right)
            when {
                tagMap.candidates(tag.id).isEmpty() -> { stats.unmappedTags++; false }
                range < config.minRange || range > config.maxRange -> { stats.outOfRangeTags++; false }
                else -> true
            }
        }
        if (usable.isEmpty()) return 0

        val cameraYaw = mount.cameraYawInField(heading, turret)
        val yawRate = cameraYawRate(t)
        var accepted = 0

        for (tag in usable) {
            val leveled = mount.levelize(tag.right, tag.down, tag.out)
            val range = hypot(leveled.x, leveled.y)
            val toTagField = Vec2(leveled.x, leveled.y).rotated(cameraYaw)
            val rayAngle = atan2(toTagField.y, toTagField.x)

            // Anisotropic measurement noise: large along the line of sight, bearing-driven across it.
            val sigmaRange = max(config.rangeSigmaFloor, config.rangeSigmaPerInchSq * range * range)
            val sigmaAngle = sqrt(
                config.pixelBearingSigma * config.pixelBearingSigma +
                config.turretAngleSigma * config.turretAngleSigma +
                config.headingSigma * config.headingSigma +
                (yawRate * config.captureTimingSigma) * (yawRate * config.captureTimingSigma)
            )
            val sigmaAcross = range * sigmaAngle
            val ux = cos(rayAngle); val uy = sin(rayAngle)
            // Treat all tags of one frame as perfectly correlated (they share heading / turret / pitch / timing errors).
            val n = usable.size.toDouble()
            val rxx = n * (sigmaRange * sigmaRange * ux * ux + sigmaAcross * sigmaAcross * uy * uy)
            val ryy = n * (sigmaRange * sigmaRange * uy * uy + sigmaAcross * sigmaAcross * ux * ux)
            val rxy = n * ((sigmaRange * sigmaRange - sigmaAcross * sigmaAcross) * ux * uy)
            val sigmaHeight = config.heightSigmaBase + config.heightSigmaPerInch * range

            val placementSigma = tagMap.placementSigma(tag.id)
            // A resting angle that is a bit off moves the tag a little: widen the noise instead of rejecting.
            val p2 = placementSigma * placementSigma
            val rxx2 = rxx + p2; val ryy2 = ryy + p2
            val sigmaHeight2 = sqrt(sigmaHeight * sigmaHeight + p2)

            val candidates = ArrayList<Candidate>()
            for ((index, placement) in tagMap.candidates(tag.id).withIndex()) {
                val solved = solve(tag, placement, heading, turret)
                // The camera's height is known. A wrong tag placement shows up as a wrong implied height.
                val heightResidual = (solved.impliedCameraHeight - mount.cameraHeight) / sigmaHeight2
                val inn = filter.innovation(t, solved.robot.x, solved.robot.y, rxx2, rxy, ryy2) ?: continue
                candidates.add(Candidate(index, solved.robot, inn.mahalanobisSq + heightResidual * heightResidual, inn.mahalanobisSq))
            }
            if (candidates.isEmpty()) continue

            val group = tagMap.group(tag.id)
            val chosen: Candidate
            if (config.useGroupBelief && group != null && candidates.size == 2) {
                // All tags of one structure share its state. Combine this tag's evidence with what earlier tags said.
                val llr = ((candidates[1].score - candidates[0].score) / 2.0)
                    .coerceIn(-config.beliefStepLimit, config.beliefStepLimit)
                val prior = beliefs[group] ?: 0.0
                val posterior = prior + llr
                if (min(candidates[0].score, candidates[1].score) <= config.gateChiSq) {
                    beliefs[group] = (prior * config.beliefDecay + llr).coerceIn(-config.beliefLimit, config.beliefLimit)
                }
                if (abs(posterior) < config.ambiguityMargin / 2.0) { stats.ambiguousTags++; continue }
                chosen = if (posterior >= 0.0) candidates[0] else candidates[1]
            } else {
                candidates.sortBy { it.score }
                chosen = candidates[0]
                if (candidates.size > 1 && candidates[1].score - chosen.score < config.ambiguityMargin) {
                    stats.ambiguousTags++
                    continue
                }
            }

            if (chosen.score > config.gateChiSq) {
                stats.gatedTags++
                consecutiveGated++
                if (consecutiveGated >= config.rejectsBeforeRecovery) {
                    filter.inflate(config.recoveryVariance)
                    consecutiveGated = 0
                }
                continue
            }
            if (filter.correct(t, chosen.robot.x, chosen.robot.y, rxx2, rxy, ryy2)) {
                accepted++
                consecutiveGated = 0
                stats.acceptedTags++
                stats.lastMahalanobisSq = chosen.mahalanobisSq
                stats.lastHypothesisIndex = chosen.index
            }
        }
        return accepted
    }
}
