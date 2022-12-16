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
import com.atomicrobotics.cflib.parallel
import com.atomicrobotics.cflib.sequential
import com.atomicrobotics.cflib.subsystems.PowerMotor
import com.atomicrobotics.cflib.subsystems.Subsystem
import com.atomicrobotics.cflib.subsystems.MotorToPosition
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
    var NAME_1 = "lift1"
    var NAME_2 = "lift2"

    // Lift Positions
    @JvmField
    var HIGH_JUNCTION = 30.5 // in
    @JvmField
    var MEDIUM_JUNCTION = 25.0 // in
    @JvmField
    var LOW_JUNCTION = 15.0 // in
    @JvmField
    var GROUND_JUNCTION = 1.5 // in
    @JvmField
    var INTAKE_POSITION = 0.0 // in
    @JvmField
    var STACK_5 = 5.0 // in
    @JvmField
    var STACK_4 = 3.75 // in
    @JvmField
    var STACK_3 = 2.5 // in
    @JvmField
    var STACK_2 = 1.25 // in
    @JvmField
    var ABOVE_STACK = 14.0 // in
    @JvmField
    var SLIGHTLY_LOWER = 27.0 // in

    // Motor Information
    @JvmField
    var DIRECTION_1 = DcMotorSimple.Direction.REVERSE
    @JvmField
    var DIRECTION_2 = DcMotorSimple.Direction.FORWARD
    @JvmField
    var SPEED_1 = 1.0
    @JvmField
    var SPEED_2 = 1.0

    // unconfigurable constants
    private const val PULLEY_WIDTH = 0.7 // in
    private const val COUNTS_PER_REV = 28 * 3.700000000 // NeveRest 20 orbital (really 19.2 ratio, not 20)
    private const val DRIVE_GEAR_REDUCTION = 1.000000000 // higher value means that driven gear is slower
    private const val COUNTS_PER_INCH = COUNTS_PER_REV * DRIVE_GEAR_REDUCTION / (PULLEY_WIDTH * Math.PI)

    // manual control
    val start: Command
        get() = parallel {
            +PowerMotor(liftMotor_1, SPEED_1, requirements = listOf(this@Lift))
            +PowerMotor(liftMotor_1, SPEED_1, requirements = listOf(this@Lift))
        }
    val reverse: Command
        get() = parallel {
            +PowerMotor(liftMotor_1, -SPEED_1, requirements = listOf(this@Lift))
            +PowerMotor(liftMotor_1, -SPEED_1, requirements = listOf(this@Lift))
        }
    val stop: Command
        get() = parallel {
            +PowerMotor(liftMotor_1, 0.0, requirements = listOf(this@Lift))
            +PowerMotor(liftMotor_1, 0.0, requirements = listOf(this@Lift))
        }

    // preset positions
    val toHigh: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (HIGH_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift))
            +MotorToPosition(liftMotor_2, (HIGH_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift))
        }
    val toMedium: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (MEDIUM_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift))
            +MotorToPosition(liftMotor_2, (MEDIUM_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift))
        }
    val toLow: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (LOW_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift))
            +MotorToPosition(liftMotor_2, (LOW_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift))
        }
    val toGround: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (GROUND_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift))
            +MotorToPosition(liftMotor_2, (GROUND_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift))
        }
    val toIntake: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (INTAKE_POSITION * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift))
            +MotorToPosition(liftMotor_2, (INTAKE_POSITION * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift))
        }
    val toLevel5: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (STACK_5 * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift))
            +MotorToPosition(liftMotor_2, (STACK_5 * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift))
        }
    val toLevel4: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (STACK_4 * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift))
            +MotorToPosition(liftMotor_2, (STACK_4 * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift))
        }
    val toLevel3: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (STACK_3 * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift))
            +MotorToPosition(liftMotor_2, (STACK_3 * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift))
        }
    val toLevel2: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (STACK_2 * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift))
            +MotorToPosition(liftMotor_2, (STACK_2 * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift))
        }
    val aboveStack: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (ABOVE_STACK * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift))
            +MotorToPosition(liftMotor_2, (ABOVE_STACK * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift))
        }
    val slightlyLower: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (SLIGHTLY_LOWER * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift))
            +MotorToPosition(liftMotor_2, (SLIGHTLY_LOWER * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift))
        }


    var conesLeftOnStack = 5
    val toNextStack: Command
        get() = sequential {
            +OptionCommand("THIS IS A TERRIBLE WORKAROUND, DON'T USE THIS.", { conesLeftOnStack }, Pair(5, toLevel5), Pair(4, toLevel4), Pair(3, toLevel3), Pair(2, toLevel2), Pair(1, toIntake))
            +CustomCommand(_start = { conesLeftOnStack-- })
        }
    // motor
    lateinit var liftMotor_1: DcMotorEx
    lateinit var liftMotor_2: DcMotorEx

    /**
     * Initializes the liftMotor, resets its encoders, sets the mode to RUN_USING_ENCODER, and sets the direction to the
     * DIRECTION variable.
     */
    override fun initialize() {
        liftMotor_1 = opMode.hardwareMap.get(DcMotorEx::class.java, NAME_1)
        liftMotor_1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor_1.mode = DcMotor.RunMode.RUN_USING_ENCODER
        liftMotor_1.direction = DIRECTION_1
        liftMotor_2 = opMode.hardwareMap.get(DcMotorEx::class.java, NAME_2)
        liftMotor_2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor_2.mode = DcMotor.RunMode.RUN_USING_ENCODER
        liftMotor_2.direction = DIRECTION_2
    }
}
