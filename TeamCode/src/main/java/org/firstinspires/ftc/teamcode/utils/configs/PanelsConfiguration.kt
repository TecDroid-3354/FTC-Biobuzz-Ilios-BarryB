package org.firstinspires.ftc.teamcode.utils.configs

import com.bylazar.panels.PanelsConfig

/**
 * Overrides the default configurations for Panels. There's no need to instantiate this class as Panels finds
 * it automatically by scanning for any class extending [com.bylazar.panels.PanelsConfig]
 */
class PanelsConfiguration: PanelsConfig() {
    override var enableClassCallerLogs: Boolean = true
    override var enableLogs: Boolean = true
}