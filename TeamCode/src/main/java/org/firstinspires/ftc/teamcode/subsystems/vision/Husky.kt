package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.seattlesolvers.solverslib.util.InterpLUT
import com.seattlesolvers.solverslib.util.get
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.units.Distance
import kotlin.math.hypot

// robotPose should be a supplier 2d, at the moment it is a Pose2d for testing with only the camera

class Husky(hardwareMap: HardwareMap, var robotPose : Pose2d = Pose2d(Translation2d(0.0,0.0),
    Rotation2d(0.0))
): SubsystemBase() {
    val husky : HuskyLens
    val interpolation = InterpLUT()
    lateinit var pollenBlock : Array<HuskyLens.Block>

    init {
        husky = hardwareMap.get(HuskyLens::class.java, HuskyConstants.Identification.huskyId)
        configureCamera()
        configureInterpolation()
    }

    override fun periodic() {
        super.periodic()

        pollenBlock = husky.blocks(HuskyConstants.Identification.pollenId)
    }





    private fun configureCamera(){

        husky.selectAlgorithm(HuskyConstants.Configuration.algorithm)
    }

    private fun configureInterpolation(){
        for (point in HuskyConstants.Interpolation.pixelToForwardDistanceMap){
            interpolation.add(point.key, point.value.meters)
        }

        interpolation.createLUT()
    }

    //These functions were created because the Husky coordinate system starts at the top left, these functions
    // transform the readings to the center.

    private fun readHuskyX() : Int{
        val xOffsets = pollenBlock.get(0).x - HuskyConstants.Mechanical.xOffset

        return xOffsets
    }
    private fun readHuskyY() : Int{
        val yOffsets = pollenBlock.get(0).y - HuskyConstants.Mechanical.yOffset
        return yOffsets
    }

    private fun getForwardDistanceToPollen() : Distance{

        val distance = Distance(interpolation.get(
             pollenBlock.get(0).width))

        return distance

    }


    //Pixel width / real distance == how long is each pixel.
    //By multiplying it by the x offset you get the real distance on meters.

    private fun getLateralDistanceToPollen() : Distance {
        return Distance.fromMeters(
            this.readHuskyX() * ( HuskyConstants.gamePieceProperties.pollenWidth.meters /pollenBlock.get(0).width)
        )
    }

    //Considers the hyp of the lateral and forward distance as the X  and the height of the camera as the Y.
    // This is from a lateral view

    private fun getPollenPoseRelativeToRobot() : Pose2d{
            val x = hypot(
                getLateralDistanceToPollen().meters,getForwardDistanceToPollen().meters)
            val y = HuskyConstants.Mechanical.huskyHeight.meters

            return Pose2d(Translation2d(x,y), Rotation2d(0.0))

    }

    private fun cameraCondition() : String{
        val cameraCondition = when(husky.knock()){
            true -> "Camera is live"
            false -> "Camera is DEAD"
        }
        return cameraCondition
    }

    fun getPollenPoseRelativeToField() : Pose2d?{
        if(isPollenDetected()){
            return getPollenPoseRelativeToRobot().relativeTo(robotPose)
        }else{
            return null
        }
    }

    fun isPollenDetected() : Boolean {
        return (pollenBlock.isNullOrEmpty()).not()
    }

    fun log(telemetry: Telemetry){
        telemetry.addLine("Camera Information")
        telemetry.addLine(cameraCondition())
        telemetry.addData("Is pollen being Detected: ", isPollenDetected())

        if(isPollenDetected()){
            telemetry.addData("Pixel Width: ", pollenBlock.get(0).width)
            telemetry.addData("Lateral Distance: ", this.getLateralDistanceToPollen().meters)
            telemetry.addData("Forward Distance:" , this.getForwardDistanceToPollen().meters)
            telemetry.addData("Distance to Camera: ",hypot(getPollenPoseRelativeToRobot().x, getPollenPoseRelativeToRobot().y))

        }



    }
}