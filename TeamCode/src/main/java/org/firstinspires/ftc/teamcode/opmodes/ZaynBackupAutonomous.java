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
    private DcMotor deadWheel = null;
    private Servo claw = null;
    private Servo arm = null;

    BNO055IMU imu;
    NormalizedColorSensor colorSensor = null;

    Orientation angles;

    final float[] hsvValues = new float[3];
    final double K = 0.05;
    final float circumferenceYJ = 3.77953f;
    final float encoderCountYJ = 537.7f;
    final float circumferenceNR40 = 12.56f;
    final float encoderCountNR40 = 1120f;
    final float circumferenceDW = 0.688975f;
    final float encoderCountDW = 8192;

    final int liftZero = 0;
    final int liftGround = 120;
    final int liftLow = 1300;
    final int liftMed = 2100;
    final int liftHigh = 2900;
    final int liftStack = 700;

    final double armFront = 0.82;
    final double armSide = 0.5;
    final double armBack = 0.15;

    final double clawOpen = 0.9;
    final double clawClosed = 0.4;

    int gain = 20;
    double error;
    double average;
    double speed;

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

        deadWheel = hardwareMap.get(DcMotor.class, "LB");

        //lift = hardwareMap.get(DcMotor.class, "lift");
        //claw = hardwareMap.get(Servo.class, "claw");
        //arm = hardwareMap.get(Servo.class, "arm");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        //LF.setDirection(DcMotor.Direction.REVERSE);
        //RF.setDirection(DcMotor.Direction.FORWARD);
        //LB.setDirection(DcMotor.Direction.REVERSE);
        //RB.setDirection(DcMotor.Direction.FORWARD);

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(parameters);

        EndLocation endLocation;

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            //closeClaw();
            forward(0, 9);
            sleep(500);
            endLocation = readSensor();
            //raise(liftLow);
            //openClaw();
            //lower();
            if (endLocation == EndLocation.LEFT){
                forward(0, 10.0);
                turn(Direction.LEFT, 90);
                forward(90, 16.0);
            }
            else if (endLocation == EndLocation.MIDDLE){
                forward(0, 7.0);
            }
            else {
                forward(0, 10.0);
                turn(Direction.RIGHT, -90);
                forward(-90, 16.0);
            }
        }
    }

    Orientation getAngles() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
    }

    public EndLocation readSensor(){
        EndLocation endLocation = EndLocation.LEFT;
        colorSensor.setGain(gain);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
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

    public void forward(double targetAngle, double targetPosition){

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double encoderPosition = (targetPosition / circumferenceDW) * encoderCountDW;
        do {
            angles = getAngles();
            //speed = Range.clip(0.00005 * (Math.abs(encoderPosition)-Math.abs(average)), 0, 1);
            speed = 0.4;
            error = K * (angles.firstAngle - targetAngle);
            LF.setPower(speed - error);
            RF.setPower(speed + error);
            LB.setPower(speed - error);
            RB.setPower(speed + error);
            telemetry.addData("Motor Positions", deadWheel.getCurrentPosition());
            telemetry.addData("Target Position", encoderPosition);
            telemetry.update();
        } while (Math.abs(deadWheel.getCurrentPosition()) < Math.abs(encoderPosition) && opModeIsActive());

        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turn (Direction direction, double targetAngle){

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (direction == Direction.RIGHT){
            LF.setPower(-0.5);
            RF.setPower(0.5);
            LB.setPower(-0.5);
            RB.setPower(0.5);
            do {
                angles = getAngles();
                telemetry.addData("Robot Angle", angles.firstAngle);
                telemetry.update();
            } while (opModeIsActive() && (angles.firstAngle) > (targetAngle));
        }
        else{
            LF.setPower(0.5);
            RF.setPower(-0.5);
            LB.setPower(0.5);
            RB.setPower(-0.5);
            do {
                angles = getAngles();
            } while (opModeIsActive() && (angles.firstAngle) < (targetAngle));
        }

        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void raise (int height){
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(height);
        lift.setPower(1);
        while (lift.isBusy() || arm.getPosition() != armSide){
            arm.setPosition(armSide);
        }
        lift.setPower(0);
    }

    public void lower (){
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(liftZero);
        lift.setPower(1);
        while (lift.isBusy() || arm.getPosition() != armFront){
            arm.setPosition(armFront);
        }
        lift.setPower(0);
    }

    public void closeClaw (){
        while (claw.getPosition() != clawClosed){
            claw.setPosition(clawClosed);
        }
    }

    public void openClaw (){
        while (claw.getPosition() != clawOpen){
            claw.setPosition(clawOpen);
        }
    }

}