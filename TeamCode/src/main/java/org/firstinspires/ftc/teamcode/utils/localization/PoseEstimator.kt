package org.firstinspires.ftc.teamcode.utils.localization

import org.firstinspires.ftc.teamcode.systems.TurretCameraMount
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
    private val config: EstimatorConfig = EstimatorConfig(),
    maxLoopHz: Double = 600.0
) {
    private companion object { const val YAW_HALF_WINDOW = 0.03 }

    private val sampleCap: Int = maxOf(64, kotlin.math.ceil(config.historySeconds * maxLoopHz).toInt())
    private val filter = PositionKalmanFilter(config.historySeconds, maxLoopHz + 100.0)

    private val sampleT = DoubleArray(sampleCap)
    private val sampleHeading = DoubleArray(sampleCap)
    private val sampleTurret = DoubleArray(sampleCap)
    val stats = EstimatorStats()
    private var sampleHead = 0
    private var sampleCount = 0

    // Scratch outputs for sample lookup & solve
    private var lookedUpHeading = 0.0
    private var lookedUpTurret = 0.0
    private var solvedRobotX = 0.0
    private var solvedRobotY = 0.0
    private var solvedImpliedHeight = 0.0

    private var hasOdometry = false
    private var lastX = 0.0
    private var lastY = 0.0
    private var lastHeading = 0.0
    private var lastT = 0.0
    private var consecutiveGated = 0
    private val groupIndex = HashMap<String, Int>()
    private var beliefArr = DoubleArray(8)

    private fun beliefSlot(group: String): Int {
        var i = groupIndex[group]
        if (i == null) {
            i = groupIndex.size
            groupIndex[group] = i
            if (i >= beliefArr.size) beliefArr = beliefArr.copyOf(beliefArr.size * 2)
        }
        return i
    }

    fun groupBelief(group: String): Double = groupIndex[group]?.let { beliefArr[it] } ?: 0.0

    val x: Double @Synchronized get() = filter.x
    val y: Double @Synchronized get() = filter.y
    val isInitialized: Boolean @Synchronized get() = filter.isInitialized

    @Synchronized
    fun resetPose() {
        hasOdometry = false
        sampleHead = 0
        sampleCount = 0
        consecutiveGated = 0
        filter.clear()
        stats.lastMahalanobisSq = 0.0
        stats.lastHypothesisIndex = -1
    }

    @Synchronized
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

        if (sampleCount == sampleCap) {
            sampleHead = (sampleHead + 1) % sampleCap
            sampleCount--
        }
        val newIdx = (sampleHead + sampleCount) % sampleCap
        sampleT[newIdx] = t
        sampleHeading[newIdx] = odoHeading
        sampleTurret[newIdx] = turretAngle
        sampleCount++

        while (sampleCount > 2 && sampleT[(sampleHead + 1) % sampleCap] < t - config.historySeconds) {
            sampleHead = (sampleHead + 1) % sampleCap
            sampleCount--
        }
        lastX = odoX; lastY = odoY; lastHeading = odoHeading; lastT = t
    }

    private fun interpolateAt(t: Double): Boolean {
        if (sampleCount == 0 || t < sampleT[sampleHead]) return false
        val lastIdx = (sampleHead + sampleCount - 1) % sampleCap
        if (t >= sampleT[lastIdx]) {
            lookedUpHeading = sampleHeading[lastIdx]; lookedUpTurret = sampleTurret[lastIdx]
            return true
        }
        var lo = 0; var hi = sampleCount - 1
        while (lo + 1 < hi) {
            val mid = (lo + hi) ushr 1
            if (sampleT[(sampleHead + mid) % sampleCap] <= t) lo = mid else hi = mid
        }
        val a = (sampleHead + lo) % sampleCap
        val b = (sampleHead + hi) % sampleCap
        val dt = sampleT[b] - sampleT[a]
        val k = if (dt > 1e-9) (t - sampleT[a]) / dt else 0.0
        lookedUpHeading = sampleHeading[a] + k * wrapRadians(sampleHeading[b] - sampleHeading[a])
        lookedUpTurret  = sampleTurret[a]  + k * wrapRadians(sampleTurret[b]  - sampleTurret[a])
        return true
    }

    private fun cameraYawRate(t: Double): Double {
        if (!interpolateAt(t - YAW_HALF_WINDOW)) return 0.0
        val ya = mount.cameraYawInField(lookedUpHeading, lookedUpTurret)
        interpolateAt(t + YAW_HALF_WINDOW)
        val yb = mount.cameraYawInField(lookedUpHeading, lookedUpTurret)
        return wrapRadians(yb - ya) / (2.0 * YAW_HALF_WINDOW)
    }

    private fun solveScalar(
        leveledX: Double, leveledY: Double, leveledZ: Double,
        placeX: Double, placeY: Double, placeZ: Double,
        cosCamYaw: Double, sinCamYaw: Double,
        cosHead: Double, sinHead: Double,
        camOffsetX: Double, camOffsetY: Double
    ) {
        val toTagFieldX = leveledX * cosCamYaw - leveledY * sinCamYaw
        val toTagFieldY = leveledX * sinCamYaw + leveledY * cosCamYaw
        val camFieldX = placeX - toTagFieldX
        val camFieldY = placeY - toTagFieldY
        solvedRobotX = camFieldX - (camOffsetX * cosHead - camOffsetY * sinHead)
        solvedRobotY = camFieldY - (camOffsetX * sinHead + camOffsetY * cosHead)
        solvedImpliedHeight = placeZ - leveledZ
    }

    @Synchronized
    fun diagnose(
        tags: List<TagObservation>, knownX: Double, knownY: Double, heading: Double, turret: Double
    ): List<TagDiagnostic> {
        val out = ArrayList<TagDiagnostic>()
        val cameraYaw = mount.cameraYawInField(heading, turret)
        val cosCamYaw = cos(cameraYaw)
        val sinCamYaw = sin(cameraYaw)
        val cosHead = cos(heading)
        val sinHead = sin(heading)
        val cosTurret = cos(turret)
        val sinTurret = sin(turret)
        val camOffsetX = mount.pivotInRobot.x + (mount.cameraInTurret.x * cosTurret - mount.cameraInTurret.y * sinTurret)
        val camOffsetY = mount.pivotInRobot.y + (mount.cameraInTurret.x * sinTurret + mount.cameraInTurret.y * cosTurret)
        val cp = cos(mount.cameraPitchUp)
        val sp = sin(mount.cameraPitchUp)

        for (tag in tags) {
            val leveledX = tag.out * cp + tag.down * sp
            val leveledY = -tag.right
            val leveledZ = tag.out * sp - tag.down * cp

            for ((index, placement) in tagMap.candidates(tag.id).withIndex()) {
                solveScalar(
                    leveledX, leveledY, leveledZ,
                    placement.x, placement.y, placement.z,
                    cosCamYaw, sinCamYaw, cosHead, sinHead,
                    camOffsetX, camOffsetY
                )
                out.add(
                    TagDiagnostic(
                        id = tag.id,
                        candidateIndex = index,
                        robotX = solvedRobotX,
                        robotY = solvedRobotY,
                        errorInches = hypot(solvedRobotX - knownX, solvedRobotY - knownY),
                        heightResidualInches = solvedImpliedHeight - mount.cameraHeight
                )
                )
            }
        }
        return out
    }

    @Synchronized
    fun addVisionFrame(frame: VisionFrame): Int {
        if (!filter.isInitialized) return 0
        val t = frame.captureTimeSeconds
        if (!interpolateAt(t)) return 0
        val heading = lookedUpHeading
        val turret = lookedUpTurret
        val yawRate = cameraYawRate(t)

        var usableCount = 0
        for (i in frame.tags.indices) {
            val tag = frame.tags[i]
            val range = hypot(tag.out, tag.right)
            if (tagMap.candidates(tag.id).isEmpty()) {
                stats.unmappedTags++
            } else if (range < config.minRange || range > config.maxRange) {
                stats.outOfRangeTags++
            } else {
                usableCount++
            }
        }
        if (usableCount == 0) return 0

        val cameraYaw = mount.cameraYawInField(heading, turret)
        val cosCamYaw = cos(cameraYaw)
        val sinCamYaw = sin(cameraYaw)
        val cosHead = cos(heading)
        val sinHead = sin(heading)
        val cosTurret = cos(turret)
        val sinTurret = sin(turret)
        val camOffsetX = mount.pivotInRobot.x + (mount.cameraInTurret.x * cosTurret - mount.cameraInTurret.y * sinTurret)
        val camOffsetY = mount.pivotInRobot.y + (mount.cameraInTurret.x * sinTurret + mount.cameraInTurret.y * cosTurret)
        val cp = cos(mount.cameraPitchUp)
        val sp = sin(mount.cameraPitchUp)
        val n = usableCount.toDouble()

        var accepted = 0
        for (i in frame.tags.indices) {
            val tag = frame.tags[i]
            val placements = tagMap.candidates(tag.id)
            if (placements.isEmpty()) continue
            val rawRange = hypot(tag.out, tag.right)
            if (rawRange < config.minRange || rawRange > config.maxRange) continue

            val leveledX = tag.out * cp + tag.down * sp
            val leveledY = -tag.right
            val leveledZ = tag.out * sp - tag.down * cp
            val range = hypot(leveledX, leveledY)

            val toTagFieldX = leveledX * cosCamYaw - leveledY * sinCamYaw
            val toTagFieldY = leveledX * sinCamYaw + leveledY * cosCamYaw
            val rayAngle = atan2(toTagFieldY, toTagFieldX)

            val sigmaRange = max(config.rangeSigmaFloor, config.rangeSigmaPerInchSq * range * range)
            val sigmaAngle = sqrt(
                config.pixelBearingSigma * config.pixelBearingSigma +
                        config.turretAngleSigma * config.turretAngleSigma +
                        config.headingSigma * config.headingSigma +
                        (yawRate * config.captureTimingSigma) * (yawRate * config.captureTimingSigma)
            )
            val sigmaAcross = range * sigmaAngle
            val ux = cos(rayAngle); val uy = sin(rayAngle)

            val rxx = n * (sigmaRange * sigmaRange * ux * ux + sigmaAcross * sigmaAcross * uy * uy)
            val ryy = n * (sigmaRange * sigmaRange * uy * uy + sigmaAcross * sigmaAcross * ux * ux)
            val rxy = n * ((sigmaRange * sigmaRange - sigmaAcross * sigmaAcross) * ux * uy)
            val sigmaHeight = config.heightSigmaBase + config.heightSigmaPerInch * range

            val placementSigma = tagMap.placementSigma(tag.id)
            val p2 = placementSigma * placementSigma
            val rxx2 = rxx + p2; val ryy2 = ryy + p2
            val sigmaHeight2 = sqrt(sigmaHeight * sigmaHeight + p2)

            // Evaluate up to 2 candidates with zero list allocations
            var bestIdx = -1; var bestScore = Double.POSITIVE_INFINITY; var bestMahal = 0.0; var bestX = 0.0; var bestY = 0.0
            var secondScore = Double.POSITIVE_INFINITY
            var c0Score = Double.POSITIVE_INFINITY; var c0Mahal = 0.0; var c0X = 0.0; var c0Y = 0.0
            var c1Score = Double.POSITIVE_INFINITY; var c1Mahal = 0.0; var c1X = 0.0; var c1Y = 0.0

            for (index in placements.indices) {
                val p = placements[index]
                solveScalar(leveledX, leveledY, leveledZ, p.x, p.y, p.z, cosCamYaw, sinCamYaw, cosHead, sinHead, camOffsetX, camOffsetY)
                val heightResidual = (solvedImpliedHeight - mount.cameraHeight) / sigmaHeight2
                val mahalSq = filter.computeInnovationSq(t, solvedRobotX, solvedRobotY, rxx2, rxy, ryy2)
                if (mahalSq.isNaN()) continue
                val score = mahalSq + heightResidual * heightResidual

                if (index == 0) { c0Score = score; c0Mahal = mahalSq; c0X = solvedRobotX; c0Y = solvedRobotY }
                else if (index == 1) { c1Score = score; c1Mahal = mahalSq; c1X = solvedRobotX; c1Y = solvedRobotY }

                if (score < bestScore) {
                    secondScore = bestScore
                    bestScore = score; bestIdx = index; bestMahal = mahalSq; bestX = solvedRobotX; bestY = solvedRobotY
                } else if (score < secondScore) {
                    secondScore = score
                }
            }
            if (bestIdx == -1) continue

            val group = tagMap.group(tag.id)
            val chosenIdx: Int; val chosenScore: Double; val chosenMahal: Double; val chosenX: Double; val chosenY: Double
            if (config.useGroupBelief && group != null && placements.size == 2 && c0Score.isFinite() && c1Score.isFinite()) {
                val llr = ((c1Score - c0Score) / 2.0).coerceIn(-config.beliefStepLimit, config.beliefStepLimit)
                val slot = beliefSlot(group)
                val prior = beliefArr[slot]
                val posterior = prior + llr
                if (min(c0Score, c1Score) <= config.gateChiSq) {
                    beliefArr[slot] = (prior * config.beliefDecay + llr).coerceIn(-config.beliefLimit, config.beliefLimit)
                }
                if (abs(posterior) < config.ambiguityMargin / 2.0) { stats.ambiguousTags++; continue }
                if (posterior >= 0.0) {
                    chosenIdx = 0; chosenScore = c0Score; chosenMahal = c0Mahal; chosenX = c0X; chosenY = c0Y
                } else {
                    chosenIdx = 1; chosenScore = c1Score; chosenMahal = c1Mahal; chosenX = c1X; chosenY = c1Y
                }
            } else {
                if (secondScore - bestScore < config.ambiguityMargin) { stats.ambiguousTags++; continue }
                chosenIdx = bestIdx; chosenScore = bestScore; chosenMahal = bestMahal; chosenX = bestX; chosenY = bestY
            }

            if (chosenScore > config.gateChiSq) {
                stats.gatedTags++
                consecutiveGated++
                if (consecutiveGated >= config.rejectsBeforeRecovery) {
                    filter.inflate(config.recoveryVariance)
                    consecutiveGated = 0
                }
                continue
            }
            if (filter.correct(t, chosenX, chosenY, rxx2, rxy, ryy2)) {
                accepted++
                consecutiveGated = 0
                stats.acceptedTags++
                stats.lastMahalanobisSq = chosenMahal
                stats.lastHypothesisIndex = chosenIdx
            }
        }
        return accepted
    }
}