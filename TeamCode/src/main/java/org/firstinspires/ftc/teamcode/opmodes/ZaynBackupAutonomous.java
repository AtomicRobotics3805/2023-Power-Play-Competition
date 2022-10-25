/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Power Play Backup OpMode", group="Linear Opmode")

public class ZaynBackupAutonomous extends LinearOpMode {

    enum EndLocation{
        LEFT,
        MIDDLE,
        RIGHT
    }

    enum Direction{
        RIGHT,
        LEFT
    }

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;

    private DcMotor lift = null;
    private Servo claw = null;
    private Servo arm = null;

    BNO055IMU imu;
    NormalizedColorSensor colorSensor;

    Orientation angles;

    final float[] hsvValues = new float[3];
    final double K = 0.05;
    final float circumference = 3.77953f;
    final float encoderCountYJ = 383.6f;
    final int junction = 3000;
    final int ground = 120;
    final double armRight = 0.4;
    final double armCenter = 0.1;
    final double open = 0.9;
    final double closed = 0.4;
    int gain = 20;
    double error;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;

        LF  = hardwareMap.get(DcMotor.class, "left_front");
        RF = hardwareMap.get(DcMotor.class, "right_front");
        LB  = hardwareMap.get(DcMotor.class, "left_back");
        RB = hardwareMap.get(DcMotor.class, "right_back");

        lift = hardwareMap.get(DcMotor.class, "lift");
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        imu.initialize(parameters);

        EndLocation endLocation;

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            forward(LF,RF,LB,RB,-0.5, 0, 19.25);
            stopAndReset(LF,RF,LB,RB);
            endLocation = readSensor(colorSensor);
            raise(lift,arm);
            openClaw(claw);
            lower(lift,arm);
        }
    }

    Orientation getAngles() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public EndLocation readSensor(NormalizedColorSensor sensor){
        EndLocation endLocation = EndLocation.LEFT;
        sensor.setGain(gain);
        NormalizedRGBA colors = sensor.getNormalizedColors();
        android.graphics.Color.colorToHSV(colors.toColor(), hsvValues);
        if (colors.blue > colors.red && colors.blue > colors.green){
            endLocation = EndLocation.RIGHT;
        }
        else if (colors.green > colors.red){
            endLocation = EndLocation.MIDDLE;
        }
        else {
            endLocation = EndLocation.LEFT;
        }
        return endLocation;
    }

    public void stopAndReset(DcMotor LF, DcMotor RF, DcMotor LB, DcMotor RB){
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void forward(DcMotor LF, DcMotor RF, DcMotor LB, DcMotor RB, double speed, double targetAngle, double targetPosition){
        angles = getAngles();
        double encoderPosition = (targetPosition / circumference) * encoderCountYJ;
        error = K * (angles.firstAngle - targetAngle);
        do {
            LF.setPower(speed + error);
            RF.setPower(speed - error);
            LB.setPower(speed + error);
            RB.setPower(speed - error);
        } while (LF.getCurrentPosition() < encoderPosition);
    }

    public void turn (DcMotor LF, DcMotor RF, DcMotor LB, DcMotor RB, Direction direction, double targetAngle){
        if (direction == Direction.RIGHT){
            LF.setPower(0.5);
            RF.setPower(-0.5);
            LB.setPower(0.5);
            RB.setPower(-0.5);
        }
        else{
            LF.setPower(-0.5);
            RF.setPower(0.5);
            LB.setPower(-0.5);
            RB.setPower(0.5);
        }
        do {
            angles = getAngles();
        } while (opModeIsActive()&& angles.firstAngle < targetAngle);
    }

    public void raise (DcMotor lift, Servo arm){
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(junction);
        while (lift.isBusy() || arm.getPosition() != armRight){
            lift.setPower(1);
            arm.setPosition(armRight);
        }
    }

    public void lower (DcMotor lift, Servo arm){
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(ground);
        while (lift.isBusy() || arm.getPosition() != armCenter){
            lift.setPower(1);
            arm.setPosition(armCenter);
        }
    }

    public void closeClaw (Servo claw){
        while (claw.getPosition() != closed){
            claw.setPosition(closed);
        }
    }

    public void openClaw (Servo claw){
        while (claw.getPosition() != open){
            claw.setPosition(open);
        }
    }

}

