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
package org.firstinspires.ftc.teamcode.ConeRanger.mechanisms

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

@Config
@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Lift : Subsystem {

    // configurable constants
    @JvmField
    var NAME_A = "lift_A"
    @JvmField
    var NAME_B = "lift_B"

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
    var DIRECTION_A = DcMotorSimple.Direction.FORWARD
    @JvmField
    var DIRECTION_B = DcMotorSimple.Direction.REVERSE
    @JvmField
    var SPEED = 1.0

    // unconfigurable constants
    private const val PULLEY_WIDTH = 2 // in
    private const val COUNTS_PER_REV = 28 * 19.2 // NeveRest 20 orbital (really 19.2 ratio, not 20)
    private const val DRIVE_GEAR_REDUCTION = 1.0 // higher value means that driven gear is slower
    private const val COUNTS_PER_INCH = COUNTS_PER_REV * DRIVE_GEAR_REDUCTION / (PULLEY_WIDTH * Math.PI)

    // manual control
    val start: Command
        get() = parallel {
            + PowerMotor(liftMotor_A, SPEED)
            + PowerMotor(liftMotor_B, SPEED)
        }
    val reverse: Command
        get() = parallel {
            + PowerMotor(liftMotor_A, -SPEED)
            + PowerMotor(liftMotor_B, -SPEED)
        }
    val stop: Command
        get() = parallel {
            + PowerMotor(liftMotor_A, 0.0)
            + PowerMotor(liftMotor_B, 0.0)
        }

    // preset positions
    val toHigh: Command
        get() = parallel {
            + MotorToPosition(liftMotor_A, (HIGH_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED)
            + MotorToPosition(liftMotor_B, (HIGH_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED)
        }
    val toMedium: Command
        get() = parallel {
            + MotorToPosition(liftMotor_A, (MEDIUM_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED)
            + MotorToPosition(liftMotor_B, (MEDIUM_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED)
        }
    val toLow: Command
        get() = parallel {
            + MotorToPosition(liftMotor_A, (LOW_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED)
            + MotorToPosition(liftMotor_B, (LOW_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED)
        }
    val toGround: Command
        get() = parallel {
            + MotorToPosition(liftMotor_A, (GROUND_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED)
            + MotorToPosition(liftMotor_B, (GROUND_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED)
        }
    val toIntake: Command
        get() = parallel {
            + MotorToPosition(liftMotor_A, (INTAKE_POSITION * COUNTS_PER_INCH).toInt(), SPEED)
            + MotorToPosition(liftMotor_B, (INTAKE_POSITION * COUNTS_PER_INCH).toInt(), SPEED)
        }
    val toLevel5: Command
        get() = parallel {
            + MotorToPosition(liftMotor_A, (STACK_5 * COUNTS_PER_INCH).toInt(), SPEED)
            + MotorToPosition(liftMotor_B, (STACK_5 * COUNTS_PER_INCH).toInt(), SPEED)
        }
    val toLevel4: Command
        get() = parallel {
            + MotorToPosition(liftMotor_A, (STACK_4 * COUNTS_PER_INCH).toInt(), SPEED)
            + MotorToPosition(liftMotor_B, (STACK_4 * COUNTS_PER_INCH).toInt(), SPEED)
        }
    val toLevel3: Command
        get() = parallel {
            + MotorToPosition(liftMotor_A, (STACK_3 * COUNTS_PER_INCH).toInt(), SPEED)
            + MotorToPosition(liftMotor_B, (STACK_3 * COUNTS_PER_INCH).toInt(), SPEED)
        }
    val toLevel2: Command
        get() = parallel {
            + MotorToPosition(liftMotor_A, (STACK_2 * COUNTS_PER_INCH).toInt(), SPEED)
            + MotorToPosition(liftMotor_B, (STACK_2 * COUNTS_PER_INCH).toInt(), SPEED)
        }
    val aboveStack: Command
        get() = parallel {
            + MotorToPosition(liftMotor_A, (ABOVE_STACK * COUNTS_PER_INCH).toInt(), SPEED)
            + MotorToPosition(liftMotor_B, (ABOVE_STACK * COUNTS_PER_INCH).toInt(), SPEED)
        }

    var conesLeftOnStack = 5
    val toNextStack: Command
        get() = sequential {
            +OptionCommand("THIS IS A TERRIBLE WORKAROUND, DON'T USE THIS.", { conesLeftOnStack }, Pair(5, toLevel5), Pair(4, toLevel4), Pair(3, toLevel3), Pair(2, toLevel2), Pair(1, toIntake))
            +CustomCommand(_start = { conesLeftOnStack-- })
        }
    // motor
        lateinit var liftMotor_A: DcMotorEx
        lateinit var liftMotor_B: DcMotorEx

    /**
     * Initializes the liftMotor, resets its encoders, sets the mode to RUN_USING_ENCODER, and sets the direction to the
     * DIRECTION variable.
     */
    override fun initialize() {
        liftMotor_A = opMode.hardwareMap.get(DcMotorEx::class.java, NAME_A)
        liftMotor_B = opMode.hardwareMap.get(DcMotorEx::class.java, NAME_B)
        liftMotor_A.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor_B.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor_A.mode = DcMotor.RunMode.RUN_USING_ENCODER
        liftMotor_B.mode = DcMotor.RunMode.RUN_USING_ENCODER
        liftMotor_A.direction = DIRECTION_A
        liftMotor_B.direction = DIRECTION_B
    }
}
