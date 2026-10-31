package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.dfrobot.HuskyLens
import com.seattlesolvers.solverslib.util.InterpLUT
import dev.frozenmilk.util.units.Unit
import dev.frozenmilk.util.units.distance.inch
import dev.frozenmilk.util.units.distance.m
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
        val pollenWidth = 2.86.inch
    }

    object Interpolation{
        //Double represents pixels, this map values the relation between the pixel width and distance in meters.
        val pixelToForwardDistanceMap = mapOf<Double, Distance>(
            0.0 to Distance.fromMeters(1.0),
            10.0 to Distance.fromMeters(1.0),
            20.0 to Distance.fromMeters(1.0),
            30.0 to Distance.fromMeters(1.0),
            40.0 to Distance.fromMeters(1.0),
            50.0 to Distance.fromMeters(1.0),
            60.0 to Distance.fromMeters(1.0),
            70.0 to Distance.fromMeters(1.0),
            80.0 to Distance.fromMeters(1.0),
            90.0 to Distance.fromMeters(1.0),
            100.0 to Distance.fromMeters(1.0),
            110.0 to Distance.fromMeters(1.0),
            120.0 to Distance.fromMeters(1.0),
            130.0 to Distance.fromMeters(1.0),
            140.0 to Distance.fromMeters(1.0),
            150.0 to Distance.fromMeters(1.0),
            160.0 to Distance.fromMeters(1.0),
            170.0 to Distance.fromMeters(1.0),
            180.0 to Distance.fromMeters(1.0),
            190.0 to Distance.fromMeters(1.0),
            200.0 to Distance.fromMeters(1.0),
            210.0 to Distance.fromMeters(1.0),
            220.0 to Distance.fromMeters(1.0),
            230.0 to Distance.fromMeters(1.0),
            240.0 to Distance.fromMeters(1.0),
            250.0 to Distance.fromMeters(1.0),
            260.0 to Distance.fromMeters(1.0),
            270.0 to Distance.fromMeters(1.0),
            280.0 to Distance.fromMeters(1.0),
            290.0 to Distance.fromMeters(1.0),
            300.0 to Distance.fromMeters(1.0),
            310.0 to Distance.fromMeters(1.0),
            320.0 to Distance.fromMeters(1.0),
            330.0 to Distance.fromMeters(1.0),
            340.0 to Distance.fromMeters(1.0),
            350.0 to Distance.fromMeters(1.0),
            360.0 to Distance.fromMeters(1.0),
        )
        val pixelToForwardDistanceInterpolation = InterpLUT()
    }


}