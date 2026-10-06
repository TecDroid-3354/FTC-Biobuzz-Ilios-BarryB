package org.firstinspires.ftc.teamcode.subsystems.vision

object LimelightConstants {

    object Identification {
        const val LIMELIGHT_ID = "limelight"
    }

    // The camera's position/rotation no longer live here (and no longer in the Limelight web UI):
    // they are in LocalizationConstants.mount, because the camera moves with the turret.

    object Configuration {
        const val POLL_RATE_HZ = 200
        const val APRIL_TAG_PIPELINE = 0
        const val STALENESS_THRESHOLD = 100

        /**
         * Added to the reported age of a frame (ms). Starts at 0. Tune it so that position fixes taken while the
         * turret/robot is rotating stop being biased in the direction of rotation (see tuning notes).
         */
        const val LATENCY_CALIBRATION_MS = 0.0
    }
}