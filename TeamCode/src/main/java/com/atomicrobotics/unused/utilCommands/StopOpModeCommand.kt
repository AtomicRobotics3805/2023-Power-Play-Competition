package com.atomicrobotics.unused.utilCommands

import com.atomicrobotics.unused.Command
import com.atomicrobotics.unused.Constants.opMode

class StopOpModeCommand: Command() {

    override val _isDone: Boolean
        get() = true

    override fun start() {
        opMode.requestOpModeStop()
    }
}