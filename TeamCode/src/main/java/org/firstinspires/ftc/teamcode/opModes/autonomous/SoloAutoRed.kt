package org.firstinspires.ftc.teamcode.opModes.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.autonomous.autos.SoloAuto
import org.firstinspires.ftc.teamcode.utils.Alliance

@Autonomous(group = "Autonomous", name = "Solo-Auto-RED")
class SoloAutoRed: SoloAuto(Alliance.RED)