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
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.atomicrobotics.cflib.Constants.opMode
import com.atomicrobotics.cflib.Command
import com.atomicrobotics.cflib.CommandScheduler
import com.atomicrobotics.cflib.hardware.MotorEx
import com.atomicrobotics.cflib.sequential
import com.atomicrobotics.cflib.subsystems.PowerMotor
import com.atomicrobotics.cflib.subsystems.Subsystem
import com.atomicrobotics.cflib.subsystems.MotorToPosition
import com.atomicrobotics.cflib.utilCommands.ConditionalCommand
import com.atomicrobotics.cflib.utilCommands.CustomCommand
import com.atomicrobotics.cflib.utilCommands.OptionCommand

/**
 * This class is an example of a lift controlled by a single motor. Unlike the Intake example object, it can use
 * encoders to go to a set position. Its first two commands, toLow and toHigh, do just that. The start command turns
 * the motor on and lets it spin freely, and the reverse command does the same but in the opposite direction. The stop
 * command stops the motor. These last three are meant for use during the TeleOp period to control the lift manually.
 * To use this class, copy it into the proper package and change the first eight constants (COUNTS_PER_INCH is fine as
 * is).
 */
@Config
@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Lift : Subsystem {

    // configurable constants
    @JvmField
    var NAME = "lift"
    @JvmField
    var MOTOR_TYPE = MotorEx.MotorType.ANDYMARK_NEVEREST
    @JvmField
    var TOTAL_GEAR_RATIO = 19.2
    @JvmField
    var PULLEY_WIDTH = 2.0

    // Lift Positions
    @JvmField
    var HIGH_JUNCTION = 34.0 // in
    @JvmField
    var MEDIUM_JUNCTION = 25.0 // in
    @JvmField
    var LOW_JUNCTION = 15.0 // in
    @JvmField
    var GROUND_JUNCTION = 1.5 // in
    @JvmField
    var INTAKE_POSITION = 0.0 // in
    @JvmField
    var STACK_5 = 5.0 // in (NOT YET TESTED)
    @JvmField
    var STACK_4 = 3.75 // in (NOT YET TESTED)
    @JvmField
    var STACK_3 = 2.5 // in (NOT YET TESTED)
    @JvmField
    var STACK_2 = 1.25 // in (NOT YET TESTED)
    @JvmField
    var ABOVE_STACK = 14.0

    // Motor Information
    @JvmField
    var DIRECTION = DcMotorSimple.Direction.FORWARD
    @JvmField
    var SPEED = 1.0

    // manual control
    val start: Command
        get() = PowerMotor(liftMotor, SPEED, requirements = listOf(this))
    val reverse: Command
        get() = PowerMotor(liftMotor, -SPEED, requirements = listOf(this))
    val stop: Command
        get() = PowerMotor(liftMotor, 0.0, requirements = listOf(this))

    // preset positions
    val toHigh: Command
        get() = MotorToPosition(liftMotor, liftMotor.inchesToTicks(HIGH_JUNCTION, PULLEY_WIDTH), SPEED, listOf(this))
    val toMedium: Command
        get() = MotorToPosition(liftMotor, liftMotor.inchesToTicks(MEDIUM_JUNCTION, PULLEY_WIDTH), SPEED, listOf(this))
    val toLow: Command
        get() = MotorToPosition(liftMotor, liftMotor.inchesToTicks(LOW_JUNCTION, PULLEY_WIDTH), SPEED, listOf(this))
    val toGround: Command
        get() = MotorToPosition(liftMotor, liftMotor.inchesToTicks(GROUND_JUNCTION, PULLEY_WIDTH), SPEED, listOf(this))
    val toIntake: Command
        get() = MotorToPosition(liftMotor, liftMotor.inchesToTicks(INTAKE_POSITION, PULLEY_WIDTH), SPEED, listOf(this))
    val toLevel5: Command
        get() = MotorToPosition(liftMotor, liftMotor.inchesToTicks(STACK_5, PULLEY_WIDTH), SPEED, listOf(this))
    val toLevel4: Command
        get() = MotorToPosition(liftMotor, liftMotor.inchesToTicks(STACK_4, PULLEY_WIDTH), SPEED, listOf(this))
    val toLevel3: Command
        get() = MotorToPosition(liftMotor, liftMotor.inchesToTicks(STACK_3, PULLEY_WIDTH), SPEED, listOf(this))
    val toLevel2: Command
        get() = MotorToPosition(liftMotor, liftMotor.inchesToTicks(STACK_2, PULLEY_WIDTH), SPEED, listOf(this))
    val aboveStack: Command
        get() = MotorToPosition(liftMotor, liftMotor.inchesToTicks(ABOVE_STACK, PULLEY_WIDTH), SPEED, listOf(this))


    var conesLeftOnStack = 5
    val toNextStack: Command
        get() = sequential {
            +OptionCommand("THIS IS A TERRIBLE WORKAROUND, DON'T USE THIS.", { conesLeftOnStack }, Pair(5, toLevel5), Pair(4, toLevel4), Pair(3, toLevel3), Pair(2, toLevel2), Pair(1, toIntake))
            +CustomCommand(_start = { conesLeftOnStack-- })
        }

    // motor
    val liftMotor = MotorEx(NAME, MOTOR_TYPE, TOTAL_GEAR_RATIO)

    /**
     * Initializes the liftMotor, resets its encoders, sets the mode to RUN_USING_ENCODER, and sets the direction to the
     * DIRECTION variable.
     */
    override fun initialize() {
        liftMotor.initialize()
        liftMotor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor.motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        liftMotor.motor.direction = DIRECTION
    }
}
