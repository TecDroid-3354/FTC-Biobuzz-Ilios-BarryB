package org.firstinspires.ftc.teamcode.opModes.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.autonomous.autos.TwoTipParkBuilder
import org.firstinspires.ftc.teamcode.utils.Alliance

@Autonomous(group = "Autonomous", name = "Auto - Red")
class AutonomousRed: TwoTipParkBuilder(Alliance.RED)