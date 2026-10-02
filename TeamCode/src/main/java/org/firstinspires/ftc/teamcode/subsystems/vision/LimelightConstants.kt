package org.firstinspires.ftc.teamcode.subsystems.vision

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

object LimelightConstants {

    object Identification {
        const val LIMELIGHT_ID = "limelight"
    }

    object Mechanical {
        val limelightPose = Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0L)
        val limelightRotation = YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)
    }

    object Configuration {
        const val POLL_RATE_HZ = 200
        const val APRIL_TAG_PIPELINE = 0
        const val STALENESS_THRESHOLD = 100
    }
}