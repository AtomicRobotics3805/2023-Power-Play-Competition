/*
    Copyright (c) 2022 Atomic Robotics (https://atomicrobotics3805.org)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see https://www.gnu.org/licenses/.
*/
package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import com.atomicrobotics.cflib.Command
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.subsystems.MoveServo
import com.atomicrobotics.cflib.subsystems.Subsystem

/**
 * This class is an example of a claw controlled by a single servo. Its first two commands, open and close, which each
 * move the claw to the corresponding position. The third command, switch, opens it if it's closed and closes it if it's
 * open. The switch command is particularly useful during TeleOp.
 * If you want, you can also use this class for other mechanisms that also involve one servo, like a delivery bucket.
 * To use this class, copy it into the proper package and change the four constants.
 */
@Config
@Suppress("PropertyName", "MemberVisibilityCanBePrivate", "unused")
object Arm : Subsystem {

    // configurable constants
    @JvmField
    var NAME = "arm"
    @JvmField
    var FORWARD = 0.0
    @JvmField
    var MIDDLE = 0.5
    @JvmField
    var BACK = 1.0
    @JvmField
    var TIME = 1.0 // the number of seconds required to move the servo from 0.0 to 1.0 (not necessarily OPEN to CLOSE)

    // commands
    val toForward: Command
        get() = MoveServo(armServo, FORWARD, TIME, listOf(this), true)
    val toMiddle: Command
        get() = MoveServo(armServo, MIDDLE, TIME, listOf(this), true)
    val toBack: Command
        get() = MoveServo(armServo, BACK, TIME, listOf(this), true)

    // servo
    private lateinit var armServo: Servo

    /**
     * Initializes the clawServo.
     */
    override fun initialize() {
        armServo = Constants.opMode.hardwareMap.get(Servo::class.java, NAME)
    }
}