package org.firstinspires.ftc.teamcode.opmodes.autonomous.backup;

import com.atomicrobotics.cflib.roadrunner.AxisDirection;
import com.atomicrobotics.cflib.roadrunner.BNO055IMUUtil;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.PowerPlayPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Power Play Backup OpMode", group="Linear Opmode")
public class ZaynBackupAutonomous extends LinearOpMode {

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
    OpenCvWebcam camera;
    PowerPlayPipeline pipeline;

    double angle;

    final float[] hsvValues = new float[3];
    final double K = 0.05;
    final float circumferenceYJ = 3.77953f;
    final float encoderCountYJ = 537.7f;
    final float circumferenceNR40 = 12.56f;
    final float encoderCountNR40 = 1120f;
    final float circumferenceDW = 4.3289539405f;
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
    int endLocation;
    double error;
    double average;
    double speed;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;

        pipeline = new PowerPlayPipeline();

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        deadWheel = hardwareMap.get(DcMotor.class, "LB");

        //lift = hardwareMap.get(DcMotor.class, "lift");
        //claw = hardwareMap.get(Servo.class, "claw");
        //arm = hardwareMap.get(Servo.class, "arm");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_X);

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(parameters);

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            forward(0, 15);
            turn(-20);
            sleep(500);
            endLocation = readCamera();
            if (endLocation == 1){
                turn(0);
                forward(0, 10.0);
                turn(0);
                forward(90, 18.0);
            }
            else if (endLocation == 2){
                turn(0);
                forward(0, 10.0);
            }
            else {
                turn(0);
                forward(0, 10.0);
                turn(-92);
                forward(-90, 18.0);
            }
        }
    }

    double getAngles() {
        return Math.toDegrees(imu.getAngularOrientation().firstAngle);
    }

    public int readCamera(){
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                pipeline = new PowerPlayPipeline();
                camera.setPipeline(pipeline);
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                sleep(5000);
                endLocation = pipeline.getParkingZone();
                camera.stopStreaming();
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Camera Status", "failed to open");
                telemetry.update();
            }
        });
        return endLocation;
    }
    /*
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
    */

    public void forward(double targetAngle, double targetPosition){

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double encoderPosition = (targetPosition / circumferenceDW) * encoderCountDW;
        do {
            angle = getAngles();
            //speed = Range.clip(K * (Math.abs(encoderPosition)-Math.abs(average)), 0, 1);
            speed = 0.3;
            error = K * (angle - targetAngle);
            LF.setPower(speed + error);
            RF.setPower(speed - error);
            LB.setPower(speed + error);
            RB.setPower(speed - error);
            telemetry.addData("Motor Positions", -deadWheel.getCurrentPosition());
            telemetry.addData("Target Position", encoderPosition);
            telemetry.addData("Robot Angle", angle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Error", error);
            telemetry.update();
        } while (Math.abs(-deadWheel.getCurrentPosition()) < Math.abs(encoderPosition) && opModeIsActive());

        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);

        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turn (double targetAngle){

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (targetAngle < 0){
            LF.setPower(0.4);
            RF.setPower(-0.4);
            LB.setPower(0.4);
            RB.setPower(-0.4);
            do {
                angle = getAngles();
                telemetry.addData("Robot Angle", angle);
                telemetry.update();
            } while (opModeIsActive() && angle > targetAngle);
        }
        else if (targetAngle > 0){
            LF.setPower(-0.4);
            RF.setPower(0.4);
            LB.setPower(-0.4);
            RB.setPower(0.4);
            do {
                angle = getAngles();
            } while (opModeIsActive() && angle < targetAngle);
        }
        else {
        }

        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);

        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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