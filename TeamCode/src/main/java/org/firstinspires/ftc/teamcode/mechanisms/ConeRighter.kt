package org.firstinspires.ftc.teamcode.mechanisms

import com.atomicrobotics.cflib.Command
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.sequential
import com.atomicrobotics.cflib.subsystems.MoveServo
import com.atomicrobotics.cflib.subsystems.Subsystem
import com.qualcomm.robotcore.hardware.Servo

object ConeRighter : Subsystem {

    // configurable constants
    @JvmField
    var NAME = "cone" // Control Port 1
    @JvmField
    var EXTENDED_POSITION = 1.0
    @JvmField
    var RETRACTED_POSITION = 0.33
    @JvmField
    var TIME = 0.7 // the number of seconds required to move the servo from 0.0 to 1.0 (not necessarily OPEN to CLOSE)

    // commands
    val extend: Command
        get() = sequential {
            +MoveServo(clawServo, EXTENDED_POSITION, TIME, listOf(Claw), false)
        }
    val retract: Command
        get() = MoveServo(clawServo, RETRACTED_POSITION, TIME, listOf(this), false)
+
    // servo
    private lateinit var clawServo: Servo

    /**
     * Initializes the clawServo.
     */
    override fun initialize() {
        clawServo = Constants.opMode.hardwareMap.get(Servo::class.java, NAME)
    }
