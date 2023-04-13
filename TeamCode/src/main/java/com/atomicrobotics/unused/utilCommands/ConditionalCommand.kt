package com.atomicrobotics.unused.utilCommands

import com.atomicrobotics.unused.Command

class ConditionalCommand(
    private val condition: () -> Boolean,
    private val trueOperation: () -> Unit,
    private val falseOperation: () -> Unit = { }) : Command() {

    override val _isDone: Boolean
        get() = true

    override fun start() {
        if(condition.invoke()) {
            trueOperation.invoke()
        } else {
            falseOperation.invoke()
        }
    }
}
