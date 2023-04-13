package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.atomicrobotics.unused.Command
import com.atomicrobotics.unused.Constants
import com.atomicrobotics.unused.subsystems.Subsystem
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA

@Config
@Suppress("Unused", "MemberVisibilityCanBePrivate")
object ColorSensor : Subsystem {
    enum class SleeveColor {
        RED,
        GREEN,
        BLUE,
        UNDETECTED
    }

    @JvmField
    var NAME = "colorSensor"
    @JvmField
    var GAIN = 20f

    private lateinit var colors: NormalizedRGBA
    private lateinit var colorSensor: NormalizedColorSensor

    var detectedColor = SleeveColor.UNDETECTED

    val color: SleeveColor
        get() = if(colors.blue > colors.red && colors.blue > colors.green)
                    SleeveColor.BLUE
                else if (colors.green > colors.red)
                    SleeveColor.GREEN
                else
                    SleeveColor.RED
    val detect: DetectCommand
        get() = DetectCommand()

    class DetectCommand : Command() {
        override val _isDone: Boolean
            get() = detectedColor != SleeveColor.UNDETECTED

        override fun start() {
            detectedColor = SleeveColor.UNDETECTED
        }

        override fun execute() {
            colors = colorSensor.normalizedColors
            detectedColor = color
        }
    }

    override fun initialize() {
        colorSensor = Constants.opMode.hardwareMap.get(NormalizedColorSensor::class.java, NAME)
        colorSensor.gain = GAIN
    }

}