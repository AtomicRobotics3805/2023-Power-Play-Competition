package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name="Interleague TeleOp")
public class InterleagueTeleOp extends LinearOpMode {
    
    ElapsedTime runtime = new ElapsedTime();
    DcMotor[] motorRef = {null, null, null, null};
    Double[] motorPowers = {0.0, 0.0, 0.0, 0.0};
    
    BNO055IMU imu;
    
    Orientation angles;
    Acceleration gravity;

    DcMotor lift1;
    DcMotor lift2;
    Servo arm;
    Servo claw;
    Servo cone;
    
    int liftZero = 0;
    int liftLow = 638;
    int liftMed = 1061;
    int liftHigh = 1481;
    int liftStack = 250;
    int liftIncrement = 55;
    int tolerance = 50;
    int stackTolerance = 20;
    
    int stackNumber = 5;
    
    double armFront = 0.99;
    double armRight = 0.65;
    double armBack = 0.3;
    double armLeft = 0;
    
    double clawOpen = 0.80;
    double clawClose = 0.42;
    
    double coneOut = 1;
    double coneIn = .33;
    
    boolean flag = false;
    boolean stackFlag = false;
    
    boolean up = false;
    boolean down = false;
    boolean test = false;
    
    double imuRotationOffset = 0;
    
    // Lift Function
    public void autoLift(int position) {
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setTargetPosition(position);
        lift2.setTargetPosition(position);
        if (lift1.getCurrentPosition() < lift1.getTargetPosition()) {
            lift1.setPower(1);
            lift2.setPower(1);
        } else {
            lift1.setPower(-1);
            lift2.setPower(-1);
        }
        flag = true;
    }
   
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        double gamepadXCord;
        double gamepadYCord;
        double gamepadPowerVal;
        double gamepadDegree;
        double robotDegree;
        double moveDegree;
        double xControl;
        double yControl;
        double power;
        
        motorRef[0] = hardwareMap.get(DcMotor.class, "LF");
        motorRef[1] = hardwareMap.get(DcMotor.class, "LB");
        motorRef[2] = hardwareMap.get(DcMotor.class, "RF");
        motorRef[3] = hardwareMap.get(DcMotor.class, "RB");
        
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        cone = hardwareMap.get(Servo.class, "cone");
        
        motorRef[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRef[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRef[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRef[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        motorRef[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motorRef[1].setDirection(DcMotorSimple.Direction.REVERSE);
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        claw.setPosition(clawOpen);
        arm.setPosition(armFront);
        cone.setPosition(coneIn);

        waitForStart();
        while (opModeIsActive()) {
            
            // Drive Code
            if (gamepad1.right_bumper) {
                power = 0.5;
            } else if (gamepad1.left_bumper) {
                power = 1;
            } else {
                power = 0.9;
            }
            
            double turnRaw = -gamepad1.right_stick_x;
            double turn  =  turnRaw*power;
            
            gamepadXCord = gamepad1.left_stick_x;
            gamepadYCord = gamepad1.left_stick_y;
            gamepadPowerVal = Range.clip(Math.hypot(gamepadXCord, gamepadYCord), 0, 1)*power;
            gamepadDegree = Math.toDegrees(Math.atan2(gamepadYCord, gamepadXCord));
            robotDegree = -getAngle();
            moveDegree = gamepadDegree - robotDegree;
            xControl = Math.cos(Math.toRadians(moveDegree))*gamepadPowerVal;
            yControl = Math.sin(Math.toRadians(moveDegree))*gamepadPowerVal;
            
            motorPowers[0] = (yControl * Math.abs(yControl)) + turn - (xControl * Math.abs(xControl));
            motorPowers[1] = (yControl * Math.abs(yControl)) + turn + (xControl * Math.abs(xControl));
            motorPowers[2] = (yControl * Math.abs(yControl)) - turn + (xControl * Math.abs(xControl));
            motorPowers[3] = (yControl * Math.abs(yControl)) - turn - (xControl * Math.abs(xControl));
            
            motorRef[0].setPower(motorPowers[0]);
            motorRef[1].setPower(motorPowers[1]);
            motorRef[2].setPower(motorPowers[2]);
            motorRef[3].setPower(motorPowers[3]);
            
            // Reset Robot Orientation
            if (gamepad1.b) {
                imuRotationOffset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
            
            // Automated Lift Code
            if (gamepad2.right_trigger < 0.5 && gamepad2.left_trigger < 0.5) {
                if (gamepad2.dpad_up) {
                    autoLift(liftHigh);
                }
                
                if (gamepad2.dpad_right && arm.getPosition() < 0.25) {
                    arm.setPosition(armBack);
                } else if (gamepad2.dpad_right && arm.getPosition() > 0.5 && arm.getPosition() < 0.8) {
                    arm.setPosition(armFront);
                } else if (gamepad2.dpad_right && arm.getPosition() > 0.25) {
                    autoLift(liftZero);
                }
                
                if (gamepad2.dpad_down && arm.getPosition() < 0.25) {
                    arm.setPosition(armBack);
                } else if (gamepad2.dpad_down && arm.getPosition() > 0.25) {
                    autoLift(liftLow);
                }
                
                if (gamepad2.dpad_left) {
                    autoLift(liftMed);
                }
                
                if (flag) {
                    if (Math.abs(lift1.getCurrentPosition() - lift1.getTargetPosition()) <= tolerance) {
                        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        flag = false;
                    }
                }
            }
            
            // Manual Lift Code
            if (gamepad2.right_trigger > 0.5) {
                if (gamepad2.dpad_up) {
                    lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lift1.setPower(0.5);
                    lift2.setPower(0.5);
                } else if (gamepad2.dpad_down) {
                    lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lift1.setPower(-0.3);
                    lift2.setPower(-0.3);
                } else {
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift1.setTargetPosition(lift1.getCurrentPosition());
                    lift2.setTargetPosition(lift1.getCurrentPosition());
                    lift1.setPower(0.1);
                    lift2.setPower(0.1);
                }
            }
            
            // Arm Power to 0
            if (gamepad1.y){
                lift1.setPower(0);
                lift2.setPower(0);
            } else if (gamepad2.y){
                lift1.setPower(0);
                lift2.setPower(0);
            }
            
            // Arm Code
            if (gamepad2.right_stick_y > 0.8) {
                arm.setPosition(armFront);
            } else if (gamepad2.right_stick_x > 0.8) {
                arm.setPosition(armLeft);
            } else if (gamepad2.right_stick_y < -0.8) {
                arm.setPosition(armBack);
            } else if (gamepad2.right_stick_x < -0.8) {
                arm.setPosition(armRight);
            }
            
            // Stack Code
            if (gamepad1.dpad_up && !up) {
                if (stackNumber < 5) {
                    stackNumber++;
                } else {
                    stackNumber = 5;
                }
                up = true;
            } else if (!gamepad1.dpad_up) {
                up = false;
            }
            
            if (gamepad1.dpad_down && !down) {
                if (stackNumber > 1) {
                    stackNumber--;
                } else {
                    stackNumber = 1;
                }
                down = true;
            } else if (!gamepad1.dpad_down) {
                down = false;
            }
            
            if (gamepad2.left_trigger > 0.5) {
                if (gamepad2.dpad_right) {
                    lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lift1.setTargetPosition(liftStack - ((5 - stackNumber) * liftIncrement));
                    lift2.setTargetPosition(liftStack - ((5 - stackNumber) * liftIncrement));
                    if (lift1.getCurrentPosition() < lift1.getTargetPosition()) {
                        lift1.setPower(0.3);
                        lift2.setPower(0.3);
                    } else {
                        lift1.setPower(-0.3);
                        lift2.setPower(-0.3);
                    }
                    stackFlag = true;
                }
            }
            
            if (stackFlag) {
                    if (Math.abs(lift1.getCurrentPosition() - lift1.getTargetPosition()) <= stackTolerance) {
                        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        stackFlag = false;
                    }
            }
            
            // Claw Code
            if (gamepad2.left_bumper || gamepad1.a) {
                claw.setPosition(clawOpen);
            } else if (gamepad2.right_bumper || gamepad1.x) {
                claw.setPosition(clawClose);
            }
            
            // Cone Righting
            if (gamepad1.left_trigger > 0.75){
                cone.setPosition(coneOut);
            } else {
                cone.setPosition(coneIn);
            }
            
            // Automation
            if (gamepad2.a) {
                claw.setPosition(clawOpen);
                arm.setPosition(armFront);
            }
            
            // Drop Test
            if (gamepad1.dpad_down && !test) {
                lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift1.setTargetPosition(lift1.getCurrentPosition() - 50);
                lift2.setTargetPosition(lift1.getCurrentPosition() - 50);
                if (lift1.getCurrentPosition() < lift1.getTargetPosition()) {
                    lift1.setPower(0.3);
                    lift2.setPower(0.3);
                } else {
                    lift1.setPower(-0.3);
                    lift2.setPower(-0.3);
                }
                test = true;
                stackFlag = true;
            } else if (!gamepad1.dpad_down) {
                test = false;
            }
            
            // Telemetry
            telemetry.addData("Lift:", lift1.getCurrentPosition());
            telemetry.addData("Stack:", stackNumber);
            telemetry.addData("lift1:", lift1.getPower());
            telemetry.addData("lift2:", lift2.getPower());
            telemetry.update();
        }
    }
    
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - imuRotationOffset;
    }
    
}

