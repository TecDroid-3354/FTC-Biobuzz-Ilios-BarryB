package org.firstinspires.ftc.teamcode.opModes.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.autonomous.builder.TestPathBuilder

import org.firstinspires.ftc.teamcode.utils.Alliance

@Autonomous(group = "Autonomous", name = "Auto - Blue")
class AutonomousBlue: TestPathBuilder(Alliance.BLUE)