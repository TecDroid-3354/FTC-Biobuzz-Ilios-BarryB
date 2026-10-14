package org.firstinspires.ftc.teamcode.constants

import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.geometry.Transform2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import org.firstinspires.ftc.teamcode.utils.units.Distance

object RobotTransformations {
    val ROBOT_TO_TURRET_2D = Transform2d(Translation2d(0.0,  0.0), Rotation2d(0.0))
}

object RobotDimensions {
    val ROBOT_WIDTH = Distance.fromInches(0.0)
    val ROBOT_LENGTH = Distance.fromInches(0.0)
    val ROBOT_HEIGHT = Distance.fromInches(0.0)
    val INTAKE_WIDTH = Distance.fromInches(0.0)
}