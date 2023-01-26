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
import com.atomicrobotics.cflib.TelemetryController
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
    var HIGH_JUNCTION = 34.0 // in
    @JvmField
    var MEDIUM_JUNCTION = 25.0 // in
    @JvmField
    var LOW_JUNCTION = 15.0 // in
    @JvmField
    var GROUND_JUNCTION = 1.5 // in
    @JvmField
    var INTAKE_POSITION = 0.5 // in
    @JvmField
    var STACK_5 = 5.5 // in
    @JvmField
    var STACK_4 = 4.25 // in
    @JvmField
    var STACK_3 = 3.0 // in
    @JvmField
    var STACK_2 = 1.75 // in
    @JvmField
    var ABOVE_STACK = 14.0 // in
    @JvmField
    var SLIGHTLY_LOWER = 31.0 // in

    // Motor Information
    @JvmField
    var DIRECTION_1 = DcMotorSimple.Direction.FORWARD
    @JvmField
    var DIRECTION_2 = DcMotorSimple.Direction.FORWARD
    @JvmField
    var SPEED_1 = 0.7
    @JvmField
    var SPEED_2 = 0.7

    // unconfigurable constants
    private const val PULLEY_RADIUS = 0.6 // in
    private const val COUNTS_PER_REV = (1.0+(46.0/17.0)) * 28.0 // GoBilda 5203 1620rpm
    private const val DRIVE_GEAR_REDUCTION = 1.0
    private const val COUNTS_PER_INCH = COUNTS_PER_REV * DRIVE_GEAR_REDUCTION / (2 * PULLEY_RADIUS * Math.PI)

    // manual control
    val start: Command
        get() = parallel {
            +PowerMotor(liftMotor_1, SPEED_1, requirements = listOf(this@Lift), logData = true)
            +PowerMotor(liftMotor_1, SPEED_1, requirements = listOf(this@Lift), logData = true)
        }
    val reverse: Command
        get() = parallel {
            +PowerMotor(liftMotor_1, -SPEED_1, requirements = listOf(this@Lift), logData = true)
            +PowerMotor(liftMotor_1, -SPEED_1, requirements = listOf(this@Lift), logData = true)
        }
    val stop: Command
        get() = parallel {
            +PowerMotor(liftMotor_1, 0.0, requirements = listOf(this@Lift), logData = true)
            +PowerMotor(liftMotor_1, 0.0, requirements = listOf(this@Lift), logData = true)
        }

    // preset positions
    val toHigh: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (HIGH_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift), logData = true, kP = 0.003)
            +MotorToPosition(liftMotor_2, (HIGH_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift), logData = true, kP = 0.003)
        }
    val toMedium: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (MEDIUM_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift), logData = true, kP = 0.003)
            +MotorToPosition(liftMotor_2, (MEDIUM_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift), logData = true, kP = 0.003)
        }
    val toLow: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (LOW_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift), logData = true, kP = 0.003)
            +MotorToPosition(liftMotor_2, (LOW_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift), logData = true, kP = 0.003)
        }
    val toGround: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (GROUND_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift), logData = true, kP = 0.003)
            +MotorToPosition(liftMotor_2, (GROUND_JUNCTION * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift), logData = true, kP = 0.003)
        }
    val toIntake: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (INTAKE_POSITION * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift), logData = true, kP = 0.003)
            +MotorToPosition(liftMotor_2, (INTAKE_POSITION * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift), logData = true, kP = 0.003)
        }
    val toLevel5: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (STACK_5 * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift), logData = true, kP = 0.003)
            +MotorToPosition(liftMotor_2, (STACK_5 * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift), logData = true, kP = 0.003)
        }
    val toLevel4: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (STACK_4 * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift), logData = true, kP = 0.003)
            +MotorToPosition(liftMotor_2, (STACK_4 * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift), logData = true, kP = 0.003)
        }
    val toLevel3: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (STACK_3 * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift), logData = true, kP = 0.003)
            +MotorToPosition(liftMotor_2, (STACK_3 * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift), logData = true, kP = 0.003)
        }
    val toLevel2: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (STACK_2 * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift), logData = true, kP = 0.003)
            +MotorToPosition(liftMotor_2, (STACK_2 * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift), logData = true, kP = 0.003)
        }
    val aboveStack: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (ABOVE_STACK * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift), logData = true, kP = 0.003)
            +MotorToPosition(liftMotor_2, (ABOVE_STACK * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift), logData = true, kP = 0.003)
        }
    val slightlyLower: Command
        get() = parallel {
            +MotorToPosition(liftMotor_1, (SLIGHTLY_LOWER * COUNTS_PER_INCH).toInt(), SPEED_1, listOf(this@Lift), logData = true, kP = 0.003)
            +MotorToPosition(liftMotor_2, (SLIGHTLY_LOWER * COUNTS_PER_INCH).toInt(), SPEED_2, listOf(this@Lift), logData = true, kP = 0.003)
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

    override fun inUsePeriodic() {
        TelemetryController.telemetry.addData("Lift Motor 1 Position (ticks)", liftMotor_1.currentPosition)
        TelemetryController.telemetry.addData("Lift Motor 2 Position (ticks)", liftMotor_2.currentPosition)
        TelemetryController.telemetry.addData("Lift Motor 1 Height (in)", liftMotor_1.currentPosition / COUNTS_PER_INCH)
        TelemetryController.telemetry.addData("Lift Motor 2 Height (in)", liftMotor_2.currentPosition / COUNTS_PER_INCH)
        TelemetryController.telemetry.addData("Lift Motor 1 Power", liftMotor_1.power)
        TelemetryController.telemetry.addData("Lift Motor 2 Power", liftMotor_2.power)
    }
}
