@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.utils.autonomous

import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.utils.Alliance

/**
 * From your paths class extend this abstract class.
 * Create your paths from the RED ALLIANCE.
 * Otherwise, the paths won't be followed properly.
 */
abstract class MirroredPoseFactory(unit: AngleUnit, alliance: Alliance) {

    protected val poseFactory: PoseFactory = when (unit) {
        AngleUnit.DEGREES -> if (alliance == Alliance.BLUE) {
            PoseFactory.degrees().mirrorY(70.75).mirrorX(70.75)
        } else { PoseFactory.degrees() }
        AngleUnit.RADIANS -> if (alliance == Alliance.BLUE) {
            PoseFactory.radians().mirrorY(70.75).mirrorX(70.75)
        } else { PoseFactory.degrees() }
    }
}