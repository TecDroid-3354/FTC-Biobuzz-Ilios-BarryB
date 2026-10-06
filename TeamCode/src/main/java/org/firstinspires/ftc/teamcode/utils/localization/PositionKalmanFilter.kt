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
class PositionKalmanFilter(private val historySeconds: Double = 1.5) {

    class Estimate(val x: Double, val y: Double, val pxx: Double, val pxy: Double, val pyy: Double)

    /** [mahalanobisSq] is the squared, covariance-weighted size of the surprise. 2 degrees of freedom. */
    class Innovation(val nx: Double, val ny: Double, val mahalanobisSq: Double)

    private class Snapshot(
        val t: Double,
        var x: Double, var y: Double,
        var pxx: Double, var pxy: Double, var pyy: Double
    )

    private val history = ArrayList<Snapshot>()

    val isInitialized: Boolean get() = history.isNotEmpty()
    val x: Double get() = history.last().x
    val y: Double get() = history.last().y
    val current: Estimate get() = history.last().let { Estimate(it.x, it.y, it.pxx, it.pxy, it.pyy) }

    fun reset(t: Double, x: Double, y: Double, variance: Double) {
        history.clear()
        history.add(Snapshot(t, x, y, variance, 0.0, variance))
    }

    /** Moves the state by the odometry displacement and adds [addedVariance] (in^2) of uncertainty. */
    fun predict(t: Double, dx: Double, dy: Double, addedVariance: Double) {
        val last = history.last()
        history.add(Snapshot(t, last.x + dx, last.y + dy, last.pxx + addedVariance, last.pxy, last.pyy + addedVariance))
        while (history.size > 2 && history[1].t < t - historySeconds) history.removeAt(0)
    }

    /** Adds uncertainty to the current state. Used to recover if odometry drifted outside the gate. */
    fun inflate(addedVariance: Double) {
        val last = history.last()
        last.pxx += addedVariance
        last.pyy += addedVariance
    }

    private fun estimateAt(t: Double): Estimate? {
        if (history.isEmpty() || t < history.first().t) return null
        if (t >= history.last().t) return current
        var i = 0
        while (i + 1 < history.size && history[i + 1].t <= t) i++
        val a = history[i]
        val b = history[i + 1]
        val k = (t - a.t) / (b.t - a.t)
        fun lerp(p: Double, q: Double) = p + k * (q - p)
        return Estimate(lerp(a.x, b.x), lerp(a.y, b.y), lerp(a.pxx, b.pxx), lerp(a.pxy, b.pxy), lerp(a.pyy, b.pyy))
    }

    /** How surprising would this measurement be? Does not change the filter. Null if [captureT] is outside the history. */
    fun innovation(captureT: Double, zx: Double, zy: Double, rxx: Double, rxy: Double, ryy: Double): Innovation? {
        val e = estimateAt(captureT) ?: return null
        val sxx = e.pxx + rxx
        val sxy = e.pxy + rxy
        val syy = e.pyy + ryy
        val det = sxx * syy - sxy * sxy
        if (det < 1e-12) return null
        val nx = zx - e.x
        val ny = zy - e.y
        val d2 = (nx * (syy * nx - sxy * ny) + ny * (-sxy * nx + sxx * ny)) / det
        return Innovation(nx, ny, d2)
    }

    /** Fuses a position measurement taken at [captureT]. Returns false if it could not be applied. */
    fun correct(captureT: Double, zx: Double, zy: Double, rxx: Double, rxy: Double, ryy: Double): Boolean {
        val cap = estimateAt(captureT) ?: return false
        val sxx = cap.pxx + rxx
        val sxy = cap.pxy + rxy
        val syy = cap.pyy + ryy
        val det = sxx * syy - sxy * sxy
        if (det < 1e-12) return false

        // S^-1
        val ixx = syy / det
        val ixy = -sxy / det
        val iyy = sxx / det
        // K = Pcap * S^-1
        val kxx = cap.pxx * ixx + cap.pxy * ixy
        val kxy = cap.pxx * ixy + cap.pxy * iyy
        val kyx = cap.pxy * ixx + cap.pyy * ixy
        val kyy = cap.pxy * ixy + cap.pyy * iyy
        val nx = zx - cap.x
        val ny = zy - cap.y
        val cx = kxx * nx + kxy * ny
        val cy = kyx * nx + kyy * ny
        // K * Pcap  (symmetric)
        val kpxx = kxx * cap.pxx + kxy * cap.pxy
        val kpxy = kxx * cap.pxy + kxy * cap.pyy
        val kpyy = kyx * cap.pxy + kyy * cap.pyy

        // Shift every snapshot after the capture time. Their covariance is Pcap + (process noise since),
        // so removing K*Pcap is exact for this F = I model.
        var insertAt = history.size
        for (i in history.indices) {
            val s = history[i]
            if (s.t > captureT) {
                if (insertAt == history.size) insertAt = i
                s.x += cx; s.y += cy
                s.pxx -= kpxx; s.pxy -= kpxy; s.pyy -= kpyy
            }
        }
        if (insertAt == history.size) {
            // Capture time is at or after the newest snapshot: the correction lands on the current state.
            val last = history.last()
            last.x += cx; last.y += cy
            last.pxx -= kpxx; last.pxy -= kpxy; last.pyy -= kpyy
        } else {
            history.add(insertAt, Snapshot(captureT, cap.x + cx, cap.y + cy, cap.pxx - kpxx, cap.pxy - kpxy, cap.pyy - kpyy))
        }
        return true
    }
}
