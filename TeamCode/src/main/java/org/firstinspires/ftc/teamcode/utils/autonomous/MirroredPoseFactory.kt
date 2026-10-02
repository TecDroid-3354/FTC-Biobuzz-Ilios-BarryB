@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.utils.autonomous

import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.utils.Alliance

abstract class MirroredPoseFactory(unit: AngleUnit, alliance: Alliance) {

    protected val poseFactory: PoseFactory = when (unit) {
        AngleUnit.DEGREES -> if (alliance == Alliance.RED) {
            PoseFactory.degrees().mirrorY(70.75).mirrorX(70.75)
        } else { PoseFactory.degrees() }
        AngleUnit.RADIANS -> if (alliance == Alliance.RED) {
            PoseFactory.radians().mirrorY(70.75).mirrorX(70.75)
        } else { PoseFactory.degrees() }
    }
}