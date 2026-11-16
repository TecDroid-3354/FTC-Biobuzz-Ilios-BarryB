package org.firstinspires.ftc.teamcode.subsystems.vision

import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Distance

// TODO Before test validate these values
object LimelightConstants {

    object Identification {
        const val LIMELIGHT_ID = "limelight"
    }

    // The camera's position/rotation no longer live here (and no longer in the Limelight web UI):
    // they are in LocalizationConstants.mount, because the camera moves with the turret.

    object Configuration {
        const val POLL_RATE_HZ = 60
        const val APRIL_TAG_PIPELINE = 0
        const val STALENESS_THRESHOLD = 100

        /**
         * Added to the reported age of a frame (ms). Starts at 0. Tune it so that position fixes taken while the
         * turret/robot is rotating stop being biased in the direction of rotation (see tuning notes).
         */
        const val LATENCY_CALIBRATION_MS = 20.0
    }

    object Mechanical {
        val LIMELIGHT_LENS_HEIGHT = Distance.fromInches(15.323206)
        val LIMELIGHT_PITCH_TO_HORIZONTAL = Angle.fromDegrees(50.0)
    }
}