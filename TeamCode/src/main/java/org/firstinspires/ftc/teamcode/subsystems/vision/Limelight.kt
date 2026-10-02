@file:Suppress("JoinDeclarationAndAssignment")

package org.firstinspires.ftc.teamcode.subsystems.vision

import androidx.core.util.Supplier
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.math.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utils.extensions.toPedroPose
import org.firstinspires.ftc.teamcode.utils.units.Angle

class Limelight(hardwareMap: HardwareMap,
                private val visionConsumer: VisionConsumer,
                private val rotationSupplier: Supplier<Rotation2d>
): SubsystemBase() {

    private var limelight: Limelight3A

    init {
        limelight = hardwareMap.get(Limelight3A::class.java, LimelightConstants.Identification.LIMELIGHT_ID)
        limelight.setPollRateHz(LimelightConstants.Configuration.POLL_RATE_HZ)
        limelight.pipelineSwitch(LimelightConstants.Configuration.APRIL_TAG_PIPELINE)
    }

    override fun periodic() {
        limelight.updateRobotOrientation(rotationSupplier.get().degrees)
        val result = limelight.latestResult

        if (result != null && result.isValid) {
            if (result.staleness > LimelightConstants.Configuration.STALENESS_THRESHOLD) return

            val robotPoseMT2 = result.botpose_MT2 ?: return
            val poseInches = robotPoseMT2.position.toUnit(DistanceUnit.INCH)

            val posePedro = Pose2d(poseInches.x, poseInches.y, robotPoseMT2.orientation.getYaw(AngleUnit.RADIANS)).toPedroPose()
            visionConsumer.accept(posePedro)
        }
    }

    fun start(): Runnable {
        return { limelight.start() }
    }

    fun pause(): Runnable {
        return { limelight.pause() }
    }

    fun interface VisionConsumer {
        fun accept(estimatedPose: Pose)
    }
}