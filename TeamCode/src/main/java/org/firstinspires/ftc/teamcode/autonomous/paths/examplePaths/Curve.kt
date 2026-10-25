package org.firstinspires.ftc.teamcode.autonomous.paths.examplePaths

import com.pedropathing.api.Paths.*
import com.pedropathing.paths.Path
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.autonomous.MirroredPoseFactory

// TODO Add an argument to each paths class. This will be the alliance.
// TODO Implement MirroredPoseFactory and pass as the first argument the angle unit being used. The second is the alliance.
class Curve(alliance: Alliance): MirroredPoseFactory(AngleUnit.DEGREES, alliance) {

    // TODO Remove the private from the start pose.
    val start = poseFactory.of(36.0, 10.0, 90.0)
    private val path1 = poseFactory.of(36.0, 58.0, 180.0)
    private val path1Control1 = poseFactory.of(76.0, 34.0, 0.0)

    fun curve(): Path = curve(start, path1Control1, path1).linear(start, path1)
}