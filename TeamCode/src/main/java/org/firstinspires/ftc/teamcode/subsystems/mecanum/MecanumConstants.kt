package org.firstinspires.ftc.teamcode.subsystems.mecanum

import org.firstinspires.ftc.teamcode.utils.units.Distance

object MecanumConstants {

    object Control {
        const val IS_BRAKE_MODE                         : Boolean   = false
    }

    object WeaverConfig {
        val CLEARANCE = Distance.fromInches(4.0)
        val MAX_DISTANCE_TO_FOLLOW_PATH = Distance.fromInches(1.0)
    }
}