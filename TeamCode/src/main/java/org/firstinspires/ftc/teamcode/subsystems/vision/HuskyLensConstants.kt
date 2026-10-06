package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.dfrobot.HuskyLens

object HuskyLensConstants {

    object Identification {
        const val HUSKY_LENS_CAMERA_ID = "husky"
    }

    object Configuration {
        val TRACKING_ALGORITHM = HuskyLens.Algorithm.OBJECT_TRACKING
    }
}