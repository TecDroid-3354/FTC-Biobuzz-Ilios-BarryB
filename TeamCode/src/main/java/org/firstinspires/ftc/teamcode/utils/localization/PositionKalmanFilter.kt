package org.firstinspires.ftc.teamcode.utils.localization

/**
 * Kalman filter over the robot's field position (x, y).
 *
 * Why only x and y: the heading is already measured by Pinpoint's IMU, and a MegaTag-style vision
 * solve is *given* a heading, so it cannot measure one. Feeding that heading back in would only
 * make the filter trust its own output. See [PoseEstimator].
 *
 * Model:  state = field position. Predict step = add the odometry displacement and grow the
 * covariance (F = I, u = odometry delta). Update step = vision position measurement (H = I).
 *
 * Latency compensation: every predicted state is kept for [historySeconds]. A vision frame is
 * compared against the state the robot was in when the picture was TAKEN, and the resulting
 * correction is applied to that snapshot and everything after it. Because F = I the correction
 * carries forward exactly, which is the same result as rewinding and replaying the odometry.
 */

class PositionKalmanFilter(
    private val historySeconds: Double = 1.5,
    maxRateHz: Double = 700.0
) {
    private val cap: Int = maxOf(64, kotlin.math.ceil(historySeconds * maxRateHz).toInt())

    private val tBuf = DoubleArray(cap)
    private val xBuf = DoubleArray(cap)
    private val yBuf = DoubleArray(cap)
    private val pxxBuf = DoubleArray(cap)
    private val pxyBuf = DoubleArray(cap)
    private val pyyBuf = DoubleArray(cap)

    private var head = 0
    private var count = 0

    // Pre-allocated scratch fields to avoid returning Estimate/Innovation objects
    private var estX = 0.0
    private var estY = 0.0
    private var estPxx = 0.0
    private var estPxy = 0.0
    private var estPyy = 0.0
    var lastInnovationMahalanobisSq = 0.0
        private set

    // PositionKalmanFilter_2.kt
    val isInitialized: Boolean get() = count > 0
    val x: Double get() = if (count == 0) 0.0 else xBuf[(head + count - 1) % cap]
    val y: Double get() = if (count == 0) 0.0 else yBuf[(head + count - 1) % cap]

    fun reset(t: Double, x: Double, y: Double, variance: Double) {
        head = 0
        count = 1
        tBuf[0] = t
        xBuf[0] = x
        yBuf[0] = y
        pxxBuf[0] = variance
        pxyBuf[0] = 0.0
        pyyBuf[0] = variance
    }

    fun clear() {
        head = 0
        count = 0
        lastInnovationMahalanobisSq = 0.0
    }

    fun predict(t: Double, dx: Double, dy: Double, addedVariance: Double) {
        if (count == cap) {
            head = (head + 1) % cap
            count--
        }
        val lastIdx = (head + count - 1) % cap
        val newIdx = (head + count) % cap
        tBuf[newIdx] = t
        xBuf[newIdx] = xBuf[lastIdx] + dx
        yBuf[newIdx] = yBuf[lastIdx] + dy
        pxxBuf[newIdx] = pxxBuf[lastIdx] + addedVariance
        pxyBuf[newIdx] = pxyBuf[lastIdx]
        pyyBuf[newIdx] = pyyBuf[lastIdx] + addedVariance
        count++

        while (count > 2 && tBuf[(head + 1) % cap] < t - historySeconds) {
            head = (head + 1) % cap
            count--
        }
    }

    fun inflate(addedVariance: Double) {
        for (i in 0 until count) {
            val idx = (head + i) % cap
            pxxBuf[idx] += addedVariance
            pyyBuf[idx] += addedVariance
        }
    }

    private fun lookupEstimateAt(t: Double): Boolean {
        if (count == 0 || t < tBuf[head]) return false
        val lastIdx = (head + count - 1) % cap
        if (t >= tBuf[lastIdx]) {
            estX = xBuf[lastIdx]; estY = yBuf[lastIdx]
            estPxx = pxxBuf[lastIdx]; estPxy = pxyBuf[lastIdx]; estPyy = pyyBuf[lastIdx]
            return true
        }
        var lo = 0
        var hi = count - 1
        while (lo + 1 < hi) {
            val mid = (lo + hi) ushr 1
            if (tBuf[(head + mid) % cap] <= t) lo = mid else hi = mid
        }
        val idxA = (head + lo) % cap
        val idxB = (head + hi) % cap
        val dt = tBuf[idxB] - tBuf[idxA]
        val k = if (dt > 1e-9) (t - tBuf[idxA]) / dt else 0.0
        estX = xBuf[idxA] + k * (xBuf[idxB] - xBuf[idxA])
        estY = yBuf[idxA] + k * (yBuf[idxB] - yBuf[idxA])
        estPxx = pxxBuf[idxA] + k * (pxxBuf[idxB] - pxxBuf[idxA])
        estPxy = pxyBuf[idxA] + k * (pxyBuf[idxB] - pxyBuf[idxA])
        estPyy = pyyBuf[idxA] + k * (pyyBuf[idxB] - pyyBuf[idxA])
        return true
    }

    fun computeInnovationSq(captureT: Double, zx: Double, zy: Double, rxx: Double, rxy: Double, ryy: Double): Double {
        if (!lookupEstimateAt(captureT)) return Double.NaN
        val sxx = estPxx + rxx
        val sxy = estPxy + rxy
        val syy = estPyy + ryy
        val det = sxx * syy - sxy * sxy
        if (!(det > 1e-12)) return Double.NaN
        val nx = zx - estX
        val ny = zy - estY
        lastInnovationMahalanobisSq = (nx * (syy * nx - sxy * ny) + ny * (-sxy * nx + sxx * ny)) / det
        return lastInnovationMahalanobisSq
    }

    /** Fuses a position measurement taken at [captureT]. Returns false if it could not be applied. */
    fun correct(captureT: Double, zx: Double, zy: Double, rxx: Double, rxy: Double, ryy: Double): Boolean {
        if (!lookupEstimateAt(captureT)) return false
        val sxx = estPxx + rxx
        val sxy = estPxy + rxy
        val syy = estPyy + ryy
        val det = sxx * syy - sxy * sxy
        if (!(det > 1e-12)) return false

        // S^-1
        val ixx = syy / det
        val ixy = -sxy / det
        val iyy = sxx / det

        // K = Pcap * S^-1
        val kxx = estPxx * ixx + estPxy * ixy
        val kxy = estPxx * ixy + estPxy * iyy
        val kyx = estPxy * ixx + estPyy * ixy
        val kyy = estPxy * ixy + estPyy * iyy

        val nx = zx - estX
        val ny = zy - estY
        val cx = kxx * nx + kxy * ny
        val cy = kyx * nx + kyy * ny

        // K * Pcap (symmetric)
        val kpxx = kxx * estPxx + kxy * estPxy
        val kpxy = kxx * estPxy + kxy * estPyy
        val kpyy = kyx * estPxy + kyy * estPyy

        // Shift every snapshot after captureT forward by the correction
        var insertPos = -1
        for (i in 0 until count) {
            val idx = (head + i) % cap
            if (tBuf[idx] > captureT) {
                if (insertPos == -1) insertPos = i
                xBuf[idx] += cx; yBuf[idx] += cy
                pxxBuf[idx] -= kpxx; pxyBuf[idx] -= kpxy; pyyBuf[idx] -= kpyy
            }
        }

        if (insertPos == -1) {
            // Capture time is at or after the newest snapshot: apply directly to the current state
            val lastIdx = (head + count - 1) % cap
            xBuf[lastIdx] += cx; yBuf[lastIdx] += cy
            pxxBuf[lastIdx] -= kpxx; pxyBuf[lastIdx] -= kpxy; pyyBuf[lastIdx] -= kpyy
        } else {
            // Insert the corrected snapshot at captureT so multi-tag frames see the updated state
            if (count == cap) {
                head = (head + 1) % cap
                count--
                insertPos = (insertPos - 1).coerceAtLeast(0)
            }
            for (i in count downTo insertPos + 1) {
                val dst = (head + i) % cap
                val src = (head + i - 1) % cap
                tBuf[dst] = tBuf[src]
                xBuf[dst] = xBuf[src]; yBuf[dst] = yBuf[src]
                pxxBuf[dst] = pxxBuf[src]; pxyBuf[dst] = pxyBuf[src]; pyyBuf[dst] = pyyBuf[src]
            }
            val insIdx = (head + insertPos) % cap
            tBuf[insIdx] = captureT
            xBuf[insIdx] = estX + cx; yBuf[insIdx] = estY + cy
            pxxBuf[insIdx] = estPxx - kpxx; pxyBuf[insIdx] = estPxy - kpxy; pyyBuf[insIdx] = estPyy - kpyy
            count++
        }
        return true
    }
}