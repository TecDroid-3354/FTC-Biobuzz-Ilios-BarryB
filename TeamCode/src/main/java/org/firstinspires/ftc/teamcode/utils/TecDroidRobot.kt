package org.firstinspires.ftc.teamcode.utils

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.Robot
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.devices.OpMotorEx
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx

abstract class TecDroidRobot(private val hardwareMap: HardwareMap): Robot() {
    val pTelemetry: TelemetryManager    = PanelsTelemetry.telemetry

    init {
        initBulkReadings()
        OpMotorEx.clearRegistry()
        OpServoEx.clearRegistry()
    }

    protected abstract fun subsystemInitialization()

    abstract fun initLoop()

    abstract fun initTeleOp()

    abstract fun preTeleOp()

    abstract fun loopTeleOp()

    abstract fun initAuto(startingPose: Pose)

    abstract fun onEnd()

    abstract fun printTelemetry(telemetryManager: TelemetryManager)

    abstract fun followPathCMD(path: Path, holdEnd: Boolean, maxPower: Double): Command

    abstract fun configureBindings()

    private fun initBulkReadings() {
        super.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL)
    }

    override fun run() {
        CommandScheduler.getInstance().run()
        OpMotorEx.updateAll()
        OpServoEx.updateAll()
        loopTeleOp()
    }
}