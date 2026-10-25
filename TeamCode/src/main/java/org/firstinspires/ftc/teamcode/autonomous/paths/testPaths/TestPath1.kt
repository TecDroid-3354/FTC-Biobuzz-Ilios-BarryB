package org.firstinspires.ftc.teamcode.autonomous.paths.testPaths

import com.pedropathing.api.Paths.*
import com.pedropathing.api.PoseFactory
import com.pedropathing.paths.Path
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.autonomous.MirroredPoseFactory

class TestPath1(alliance: Alliance): MirroredPoseFactory(AngleUnit.DEGREES, alliance) {

    val start = poseFactory.of(9.8, 103.7206, 90.0)
    private val path1 = poseFactory.of(9.8, 131.3044, 90.0)
    private val point2 = poseFactory.of(35.375, 125.3515, -20.0)
    private val point3 = poseFactory.of(127.35, 94.9706, 0.0)
    private val point3Control1 = poseFactory.of(93.9765, 120.2471, 0.0)
    private val point4 = poseFactory.of(94.4721, 74.0231, 0.0)
    private val point4Control1 = poseFactory.of(114.375, 75.6647, 0.0)
    private val point5 = poseFactory.of(14.5662, 74.4392, 0.0)
    private val point6 = poseFactory.of(47.0279, 125.2128, 90.0)
    private val point7 = poseFactory.of(105.7088, 124.7966, 90.0)

    fun path1(): Path = line(start, path1).constant(path1)

    fun path2(): Path = line(path1, point2).linear(path1, point2)

    fun path3(): Path = curve(point2, point3Control1, point3).linear(point2, point3)

    fun path4(): Path = curve(point3, point4Control1, point4).constant(point4)

    fun path5(): Path = line(point4, point5).constant(point5)

    fun path6(): Path = line(point5, point6).linear(point5, point6)

    fun path7(): Path = line(point6, point7).constant(point7)
}