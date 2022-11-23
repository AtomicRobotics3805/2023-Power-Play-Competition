package com.atomicrobotics.cflib.utilCommands

import com.atomicrobotics.cflib.Command
import com.atomicrobotics.cflib.Constants.opMode

class StopOpModeCommand: Command() {

    override val _isDone: Boolean
        get() = true

    override fun start() {
        opMode.requestOpModeStop()
    }
}