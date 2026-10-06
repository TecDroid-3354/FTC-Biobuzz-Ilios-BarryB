package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase

@Suppress("JoinDeclarationAndAssignment")
class HuskyLens(hardwareMap: HardwareMap): SubsystemBase() {

    private var huskyLens: HuskyLens

    init {
        huskyLens = hardwareMap.get(HuskyLens::class.java, HuskyLensConstants.Identification.HUSKY_LENS_CAMERA_ID)
        huskyLens.selectAlgorithm(HuskyLensConstants.Configuration.TRACKING_ALGORITHM)
    }
}