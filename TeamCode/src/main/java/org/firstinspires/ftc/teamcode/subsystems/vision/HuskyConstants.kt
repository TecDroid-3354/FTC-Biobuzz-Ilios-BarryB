package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.dfrobot.HuskyLens
import com.seattlesolvers.solverslib.util.InterpLUT
import org.firstinspires.ftc.teamcode.utils.units.Distance

object HuskyConstants {
    object Identification{
        val huskyId = "huskyCamera"

        val pollenId = 1
    }

    object Configuration{
        val algorithm = HuskyLens.Algorithm.OBJECT_TRACKING
    }

    object  Mechanical{
        val huskyHeight = Distance.fromMeters(0.185) // During Testing this was the height

        val xOffset = 160
        val yOffset = 120
    }

    object gamePieceProperties {
        val pollenWidth = Distance(0.072644) // 2.86 INCHES
    }

    object Interpolation{
        //Double represents pixels, this map values the relation between the pixel width and distance in meters.
        val pixelToForwardDistanceMap = mapOf<Double, Distance>(
            0.0 to Distance.fromMeters(1.0),
            30.0 to Distance.fromMeters(0.19),
            60.0 to Distance.fromMeters(1.0),
            90.0 to Distance.fromMeters(1.0),
            120.0 to Distance.fromMeters(1.0),
            150.0 to Distance.fromMeters(1.0),
            180.0 to Distance.fromMeters(1.0),
            210.0 to Distance.fromMeters(1.0),
            240.0 to Distance.fromMeters(1.0),
            270.0 to Distance.fromMeters(1.0),
            300.0 to Distance.fromMeters(1.0),
            330.0 to Distance.fromMeters(1.0),
            360.0 to Distance.fromMeters(1.0),
        )
    }


}