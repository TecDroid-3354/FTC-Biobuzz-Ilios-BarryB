package org.firstinspires.ftc.teamcode.autonomous.paths

import com.pedropathing.api.Paths.*
import com.pedropathing.paths.Path
import com.pedropathing.paths.interpolator.Interpolator
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.autonomous.MirroredPoseFactory

class TwoTipParkPaths(alliance: Alliance): MirroredPoseFactory(AngleUnit.DEGREES, alliance) {

    val start = poseFactory.of(58.0, 9.0, 180.0)
    private val point1 = poseFactory.of(15.0, 9.0, 180.0)
    private val point2 = poseFactory.of(27.6239, 9.0, 180.0)
    private val point3 = poseFactory.of(15.0, 10.5, 180.0)
    private val point3Control1 = poseFactory.of(21.6981, 11.6807, 0.0)
    private val point3Segment1Start = poseFactory.of(15.0, 10.5, 180.0)
    private val point3Segment1End = poseFactory.of(15.0, 10.5, 200.0)
    private val point3Segment2Start = poseFactory.of(15.0, 10.5, 200.0)
    private val point3Segment2End = poseFactory.of(15.0, 10.5, 180.0)
    private val point4 = poseFactory.of(58.5, 124.3584, 90.0)
    private val point4Control1 = poseFactory.of(41.7242, 125.116, 0.0)
    private val point4Segment1Start = poseFactory.of(58.5, 124.3584, 180.0)
    private val point4Segment1End = poseFactory.of(58.5, 124.3584, 80.0)
    private val point4Segment2Start = poseFactory.of(58.5, 124.3584, 80.0)
    private val point4Segment2End = poseFactory.of(58.5, 124.3584, 90.0)
    private val point5 = poseFactory.of(51.0, 113.0, 90.0)
    private val point6 = poseFactory.of(51.0, 126.29, 90.0)
    private val point7 = poseFactory.of(51.0, 118.0, 90.0)
    private val point8 = poseFactory.of(51.0, 127.29, 90.0)
    private val point9Start = poseFactory.of(51.0, 127.29, 90.0)
    private val point9 = poseFactory.of(16.0, 120.0, 0.0)
    private val point9Control1 = poseFactory.of(37.0, 120.0, 0.0)

    fun path1(): Path = line(start, point1).linear(start, point1)

    fun path2(): Path = line(point1, point2).reverseTangent()

    fun path3(): Path = curve(point2, point3Control1, point3).heading(Interpolator.piecewise().until(0.3, Interpolator.linear(point3Segment1Start, point3Segment1End)).until(1.0, Interpolator.linear(point3Segment2Start, point3Segment2End)))

    fun path4(): Path = curve(point3, point4Control1, point4).heading(Interpolator.piecewise().until(0.3, Interpolator.linear(point4Segment1Start, point4Segment1End)).until(1.0, Interpolator.linear(point4Segment2Start, point4Segment2End)))

    fun path5(): Path = line(point4, point5).constant(point5)

    fun path6(): Path = line(point5, point6).constant(point6)

    fun path7(): Path = line(point6, point7).reverseTangent()

    fun path8(): Path = line(point7, point8).constant(point8)

    fun path9(): Path = curve(point9Start, point9Control1, point9).linear(point9Start, point9)
}
