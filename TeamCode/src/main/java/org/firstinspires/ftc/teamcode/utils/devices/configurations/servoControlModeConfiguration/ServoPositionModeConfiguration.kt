package org.firstinspires.ftc.teamcode.utils.devices.configurations.servoControlModeConfiguration

import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.devices.controlModes.ServoControlMode

class ServoPositionModeConfiguration: ServoControlModeConfiguration {

    override val controlMode: ServoControlMode
        get() = ServoControlMode.POSITION

    var range                   : ClosedRange<Angle>                 = Angle(0.0)..Angle(1.0)

    var inverted                : Boolean               = false

    fun withRange(value: ClosedRange<Angle>)   : ServoPositionModeConfiguration {
        this.range = value
        return this
    }

    fun withInverted(value: Boolean)            : ServoPositionModeConfiguration {
        this.inverted = value
        return this
    }
}