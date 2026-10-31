package org.firstinspires.ftc.teamcode.subsystems.vision

import androidx.core.util.Supplier
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.seattlesolvers.solverslib.geometry.relativeTo
import com.seattlesolvers.solverslib.util.get
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.units.Distance
import kotlin.math.hypot

// robotPose should be a supplier 2d, at the moment it is a Pose2d for testing with only the camera

class Husky(hardwareMap: HardwareMap, var robotPose : Pose2d = Pose2d(Translation2d(0.0,0.0),
    Rotation2d(0.0))
) {
    val husky : HuskyLens
    init {
        husky = hardwareMap.get(HuskyLens::class.java, HuskyConstants.Identification.huskyId)
        configureCamera()
        configureInterpolation()
    }

    fun configureCamera(){
        husky.selectAlgorithm(HuskyConstants.Configuration.algorithm)
    }

    private fun getForwardDistanceToPollen() : Distance{
        return Distance(HuskyConstants.Interpolation.pixelToForwardDistanceInterpolation.get(
            husky.blocks(HuskyConstants.Identification.pollenId).get(0).width)
        )

    }

    //Theses functions were created because the Husky coordinate system starts at the top left, these functions
    // transform the readings to the center.

    private fun readHuskyX() : MutableList<Int>{
        val xOffsets = mutableListOf<Int>()
        for(detected in husky.blocks()){
            xOffsets.add(
                 detected.x - HuskyConstants.Mechanical.xOffset
            )
        }
        return xOffsets
    }
    private fun readHuskyY() : MutableList<Int>{
        val yOffsets = mutableListOf<Int>()
        for(detected in husky.blocks()){
            yOffsets.add(
                detected.y - HuskyConstants.Mechanical.yOffset
            )
        }
        return yOffsets
    }

    //Pixel width / real distance == how long is each pixel.
    //By multiplying it by the x offset you get the real distance on meters.

    private fun getLateralDistanceToPollen() : Distance {
        return Distance.fromMeters(
            this.readHuskyX().get(0) * (husky.blocks(HuskyConstants.Identification.pollenId).get(0).width / HuskyConstants.gamePieceProperties.pollenWidth.intoMeters().value)
        )
    }

    //Considers the hyp of the lateral and forward distance as the X  and the height of the camera as the Y.
    // This is from a lateral view

    private fun getPollenPoseRelativeToRobot() : Pose2d {
        val x = hypot(getLateralDistanceToPollen().meters,getForwardDistanceToPollen().meters)
        val y = HuskyConstants.Mechanical.huskyHeight.meters

        return Pose2d(Translation2d(x,y), Rotation2d(0.0))

    }

    private fun configureInterpolation(){
        for (point in HuskyConstants.Interpolation.pixelToForwardDistanceMap){
            HuskyConstants.Interpolation.pixelToForwardDistanceInterpolation.add(point.key, point.value.meters)
        }
    }

    fun getPollenPoseRelativeToField() : Pose2d{
        return getPollenPoseRelativeToRobot().relativeTo(robotPose)
    }

    fun cameraCondition() : String{
        val cameraCondition = when(husky.knock()){
            true -> "Camera is DEAD"
            false -> "Camera is live"
        }

        return cameraCondition
    }

    fun log(telemetry: Telemetry){
        telemetry.addLine("Camera Information")
        telemetry.addLine(cameraCondition())
        telemetry.addData("Pixel Width: ", husky.blocks(HuskyConstants.Identification.pollenId).get(0).width)
        telemetry.addLine("////////")

        telemetry.addData("Lateral Offset: ", this.getLateralDistanceToPollen())
        telemetry.addData("Forward Distance: ", this.getForwardDistanceToPollen())

        telemetry.addData("Distance to Camera: ",hypot(getPollenPoseRelativeToRobot().x, getPollenPoseRelativeToRobot().y))

    }
}