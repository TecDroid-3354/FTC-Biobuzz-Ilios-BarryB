package org.firstinspires.ftc.teamcode.constants

import com.pedropathing.math.Pose
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.autonomous.MirroredPoseFactory
import org.firstinspires.ftc.teamcode.utils.extensions.toPose2d
import org.firstinspires.ftc.teamcode.utils.extensions.toTranslation2d
import org.firstinspires.ftc.teamcode.utils.units.Distance

/**
 * Creates two [Pose]s in the field, one for the right and one for the left HIVE.
 * Implements [MirroredPoseFactory] and creates a [com.pedropathing.api.PoseFactory] and mirrors
 * the poses depending on the alliance. The [Pose] is in Pedro Pathing Coordinates. When turned into a [Translation2d]
 * it converts it to standard FTC Coordinates first
 */
class ScoringTargets(alliance: Alliance): MirroredPoseFactory(AngleUnit.DEGREES, alliance) {

    val rightHivePose: Pose = poseFactory.of(0.0, 0.0, 0.0)
    val leftHivePose: Pose = poseFactory.of(0.0, 0.0, 0.0)
}

object FieldDimensions {
    val X_AXIS_WIDTH = Distance.fromInches(141.5)
    val Y_AXIS_HEIGHT = Distance.fromInches(141.5)

    val LEFT_SIDE_END = Distance.fromInches(0.0)
}