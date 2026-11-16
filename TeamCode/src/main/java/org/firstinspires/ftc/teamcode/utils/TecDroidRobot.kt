package org.firstinspires.ftc.teamcode.utils

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.Robot
import org.firstinspires.ftc.teamcode.utils.devices.OpMotorEx
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx

abstract class TecDroidRobot(hardwareMap: HardwareMap): Robot() {
    val pTelemetry: TelemetryManager = PanelsTelemetry.telemetry
    private val allHubs: List<LynxModule> = hardwareMap.getAll(LynxModule::class.java)

    init {
        initBulkReadings()
        OpMotorEx.clearRegistry()
        OpServoEx.clearRegistry()
        pTelemetry.setUpdateInterval(50)
    }
    abstract fun preAuto(command: Command)
    abstract fun initAuto(startingPose: Pose)
    abstract fun initLoop()
    abstract fun initTeleOp(autoEndPose: Pose)
    abstract fun preTeleOp()
    abstract fun loop()
    abstract fun onEnd()
    abstract fun printTelemetry(telemetryManager: TelemetryManager)
    abstract fun followPathCMD(path: Path, holdEnd: Boolean, maxPower: Double): Command
    abstract fun configureBindings()
    private fun initBulkReadings() {
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
    }

    override fun run() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }
        CommandScheduler.getInstance().run()
        OpMotorEx.updateAll()
        OpServoEx.updateAll()
        loop()
    }
}