package com.atomicrobotics.cflib.examples

import com.atomicrobotics.cflib.Command
import com.atomicrobotics.cflib.Constants.drive
import com.atomicrobotics.cflib.Constants.opMode
import com.atomicrobotics.cflib.subsystems.Subsystem
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import java.util.concurrent.TimeUnit
import kotlin.math.pow
import kotlin.math.sqrt

object AlignmentLEDFlashExample : Subsystem {

    @JvmField
    var LED_NAME = "LED"
    @JvmField
    var FLASH_DURATION = 1.0
    @JvmField
    var ERROR_THRESHOLD = 5.0 // maximum total distance we can be from the target position in inches

    private var blinkinLedDriver: RevBlinkinLedDriver? = null

    override fun initialize() {
        blinkinLedDriver = opMode.hardwareMap.get(RevBlinkinLedDriver::class.java, LED_NAME)
    }

    class TestAlignment : Command() {

        override val _isDone = false

        private val deadline = Deadline((FLASH_DURATION * 1_000_000).toLong(), TimeUnit.NANOSECONDS)
        private var on = false

        override fun execute() {
            requireNotNull(blinkinLedDriver) { "Did not initialize RevBlinkinLEDDriver" }
            val difference = (drive.startPose.invoke() - drive.poseEstimate).vec()
            if (sqrt(difference.x.pow(2) + difference.y.pow(2)) > ERROR_THRESHOLD) {
                if (deadline.hasExpired()) {
                    if (on) {
                        blinkinLedDriver!!.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED)
                    } else {
                        blinkinLedDriver!!.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
                    }
                    on = !on
                    deadline.reset()
                }
            }
        }
    }
}