package org.firstinspires.ftc.teamcode.autonomous.paths

import com.pedropathing.api.Paths
import com.pedropathing.paths.Path
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.autonomous.MirroredPoseFactory

class SoloPath(alliance : Alliance) : MirroredPoseFactory(AngleUnit.DEGREES, alliance) {
    val start = poseFactory.of(56.0, 8.0, 180.0)
    private val path1 = poseFactory.of(12.0, 8.0, 180.0)
    private val point2Start = poseFactory.of(12.0, 8.0, 180.0)
    private val point2 = poseFactory.of(19.9358, 40.665, 82.0)
    private val point3 = poseFactory.of(37.4372, 114.321, 51.9342)
    private val point3Control1 = poseFactory.of(28.5395, 102.9594, 0.0)
    private val point4 = poseFactory.of(47.2724, 126.6943, 90.0)
    private val point5 = poseFactory.of(61.0, 125.0, 90.0)
    private val point6Start = poseFactory.of(61.0, 125.0, 90.0)
    private val point6 = poseFactory.of(61.0, 22.0, 90.0)
    private val point7Start = poseFactory.of(61.0, 22.0, 90.0)
    private val point7 = poseFactory.of(14.7, 47.5897, 180.0)
    private val point7Control1 = poseFactory.of(38.8, 42.0227, 0.0)
    private val point8Start = poseFactory.of(14.7, 47.5897, 180.0)
    private val point8 = poseFactory.of(9.57, 113.3692, 90.0)
    private val point8Control1 = poseFactory.of(16.0935, 63.7711, 0.0)

    fun path1(): Path = Paths.line(start, path1).linear(start, path1)

    fun path2(): Path = Paths.line(point2Start, point2).linear(point2Start, point2)

    fun path3(): Path = Paths.curve(point2, point3Control1, point3).tangent()

    fun path4(): Path = Paths.line(point3, point4).constant(point4)

    fun path5(): Path = Paths.line(point4, point5).constant(point5)

    fun path6(): Path = Paths.line(point6Start, point6).linear(point6Start, point6)

    fun path7(): Path = Paths.curve(point7Start, point7Control1, point7).linear(point7Start, point7)

    fun path8(): Path = Paths.curve(point8Start, point8Control1, point8).linear(point8Start, point8)
}
