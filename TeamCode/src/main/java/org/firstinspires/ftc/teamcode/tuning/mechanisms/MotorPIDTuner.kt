package org.firstinspires.ftc.teamcode.tuning.mechanisms

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.atomicrobotics.cflib.*
import com.atomicrobotics.cflib.subsystems.MotorToPosition
import com.atomicrobotics.cflib.subsystems.PowerMotor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx

/**
 * Untested
 */
@Suppress("PropertyName", "PrivatePropertyName")
@Autonomous(name = "Motor PID Tuner", group = "Test")
class MotorPIDTuner : LinearOpMode() {

    @JvmField
    var COEFFICIENTS = PIDCoefficients(0.005, 0.0, 0.0)
    @JvmField
    var TARGET_POSITION = 10.0 // in
    @JvmField
    var NAME = "lift"

    @JvmField
    var PULLEY_WIDTH = 2 // in
    @JvmField
    var COUNTS_PER_REV = 28 * 19.2 // NeveRest 20 orbital (really 19.2 ratio, not 20)
    @JvmField
    var DRIVE_GEAR_REDUCTION = 1.0 // higher value means that driven gear is slower
    private val COUNTS_PER_INCH: Double
        get() = COUNTS_PER_REV * DRIVE_GEAR_REDUCTION / (PULLEY_WIDTH * Math.PI)

    val moveMotor: Command
        get() = MotorToPosition(motor, (TARGET_POSITION * COUNTS_PER_INCH).toInt(), 1.0,
            coefficients = COEFFICIENTS, finish = false)
    val moveMotorToStart: Command
        get() = MotorToPosition(motor, 0, 0.5)

    lateinit var motor: DcMotorEx
    lateinit var gamepad: GamepadEx

    override fun runOpMode() {
        Constants.opMode = this
        gamepad = GamepadEx(gamepad1)
        CommandScheduler.registerSubsystems(TelemetryController)
        TelemetryController.telemetry.addLine("Initializing")
        TelemetryController.telemetry.update()
        var tuningMotor = false
        motor = hardwareMap.get(DcMotorEx::class.java, NAME)
        TelemetryController.telemetry.addLine("Ready to Start")
        TelemetryController.telemetry.update()
        waitForStart()
        while (opModeIsActive()) {
            if (gamepad.a.pressed) {
                CommandScheduler.cancelAll()
                CommandScheduler.scheduleCommand(moveMotor)
                tuningMotor = true
            }
            if (gamepad.b.pressed) {
                CommandScheduler.cancelAll()
                CommandScheduler.scheduleCommand(moveMotorToStart)
                tuningMotor = false
            }
            if (tuningMotor) {
                TelemetryController.telemetry.addData("Current Motor Position", motor.currentPosition)
                TelemetryController.telemetry.addData("Target Motor Position", motor.targetPosition)
                TelemetryController.telemetry.addLine("Press B to move the motor back to its starting position")
            } else {
                TelemetryController.telemetry.addLine("Press A to move the motor")
            }
            CommandScheduler.run()
        }
    }
}