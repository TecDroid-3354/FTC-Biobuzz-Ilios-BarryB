@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.utils.extensions

import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.command.button.Trigger
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import com.seattlesolvers.solverslib.gamepad.whileActiveOnce

// Gamepad buttons
fun GamepadEx.a()               : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.A)           ;    }
fun GamepadEx.b()               : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.B)           ;    }
fun GamepadEx.x()               : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.X)           ;    }
fun GamepadEx.y()               : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.Y)           ;    }
fun GamepadEx.triangle()        : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.TRIANGLE)    ;    }
fun GamepadEx.circle()          : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.CIRCLE)      ;    }
fun GamepadEx.cross()           : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.CROSS)       ;    }
fun GamepadEx.square()          : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.SQUARE)      ;    }
fun GamepadEx.start()           : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.START)       ;    }
fun GamepadEx.back()            : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.BACK)        ;    }
fun GamepadEx.dpadDown()        : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)   ;    }
fun GamepadEx.dpadUp()          : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.DPAD_UP)     ;    }
fun GamepadEx.dpadRight()       : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)  ;    }
fun GamepadEx.dpadLeft()        : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)   ;    }
fun GamepadEx.rightBumper()     : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);    }
fun GamepadEx.leftBumper()      : GamepadButton     { return this.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER) ;    }
fun GamepadEx.rightTrigger()    : Trigger           { return Trigger{ this.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1 } }
fun GamepadEx.rightTrigger(threshold: Double)       : Trigger { return Trigger{ this.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > threshold } }
fun GamepadEx.leftTrigger()     : Trigger           { return Trigger{ this.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 } }
fun GamepadEx.leftTrigger(threshold: Double)        : Trigger { return Trigger{ this.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > threshold } }

fun GamepadEx.rumble(blips: Int): Command           { return InstantCommand({ this.gamepad.rumbleBlips(blips) }) }

// Gamepad buttons commands'
fun GamepadButton.onTrue    (command: Command): GamepadButton  { this.whenPressed              (command); return this   }
fun GamepadButton.whileTrue (command: Command): GamepadButton  { this.whileActiveOnce          (command); return this   }
fun GamepadButton.onFalse   (command: Command): GamepadButton  { this.whenInactive             (command); return this   }
fun Trigger.onTrue          (command: Command): Trigger        { this.whenActive               (command); return this   }
fun Trigger.whileTrue       (command: Command): Trigger        { this.whileActiveOnce          (command); return this   }
fun Trigger.onFalse         (command: Command): Trigger        { this.whenInactive             (command); return this   }