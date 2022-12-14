package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp
@Disabled
class EncoderTest: LinearOpMode() {
    override fun runOpMode() {
        val parallel = hardwareMap.get(DcMotor::class.java, "LB")
        val perp = hardwareMap.get(DcMotor::class.java, "LF")
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("Parallel Encoder Count", parallel.currentPosition)
            telemetry.addData("Perpendicular Encoder Count", perp.currentPosition)
            telemetry.update()
        }

    }
}