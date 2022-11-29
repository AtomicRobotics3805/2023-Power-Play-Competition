package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
@TeleOp(name="Field Centric TeleOp v2", group="DriveTrains")
public class FieldCentricTeleOpv2 extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final DcMotor[] motorRef = {null, null, null, null};
    private final Double[] motorPowers = {0.0, 0.0, 0.0, 0.0};
    
    public BNO055IMU imu;
    
    Orientation angles;
    Acceleration gravity;
    
    //Mechanisms
   private DcMotor lift;
   private Servo arm;
   private Servo claw;
   
   //Positions
   int liftZero = 0;
   int liftGround = 120;
   int liftLow = 1300;
   int liftMed = 2100;
   int liftHigh = 2900;
   int liftStack = 700;
   
   double armFront = 0.82;
   double armSide = 0.5;
   double armBack = 0.15;
   
   double clawOpen = 0.80;
   double clawClose = 0.42;
   double clawBeacon = 0.6;
   
   public double armResetTimer = 0.0;
   public double armoldTime = 0.0;
   public boolean armCount = false;
   public int stackIncrement = 0;
   public int stackModifier = 0;
   
   boolean liftMode = false; //false = run to position mode.
   DcMotor.RunMode liftRunMode = DcMotor.RunMode.RUN_TO_POSITION;


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
        

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorRef[0] = hardwareMap.get(DcMotor.class, "LF");
        motorRef[1] = hardwareMap.get(DcMotor.class, "LB");
        motorRef[2] = hardwareMap.get(DcMotor.class, "RF");
        motorRef[3] = hardwareMap.get(DcMotor.class, "RB");
        
        //Mechanisms
       lift = hardwareMap.get(DcMotor.class, "lift");
       arm = hardwareMap.get(Servo.class, "arm");
       claw = hardwareMap.get(Servo.class, "claw");
       

       lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       lift.setTargetPosition(0);
       lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       
       motorRef[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       motorRef[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       motorRef[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       motorRef[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       
       lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       
       
       claw.setPosition(clawOpen);
       arm.setPosition(armFront);
       

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        int iterator = 0;
        motorRef[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motorRef[1].setDirection(DcMotorSimple.Direction.REVERSE);
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

 imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                power = 0.6;
            } else if (gamepad1.left_bumper) {
                power = 1;
            } else {
                power = 0.9;
            }
            double turnRaw = -gamepad1.right_stick_x;
            double turn  =  turnRaw*power;
            //double strafe = gamepad1.left_stick_x;
            
            gamepadXCord = gamepad1.left_stick_x;
            gamepadYCord = gamepad1.left_stick_y;
            gamepadPowerVal = Range.clip(Math.hypot(gamepadXCord, gamepadYCord), 0, 1)*power;
            gamepadDegree = Math.toDegrees(Math.atan2(gamepadYCord, gamepadXCord)); //Here was one of the problems, the output was in radians so I converted it to degrees
            robotDegree = -getAngle();//Here was a second problem, the angle was somehow reverced so I added the "-" to fix it
            moveDegree = gamepadDegree - robotDegree;
            xControl = Math.cos(Math.toRadians(moveDegree))*gamepadPowerVal;
            yControl = Math.sin(Math.toRadians(moveDegree))*gamepadPowerVal;
            
            motorPowers[0] = (yControl * Math.abs(yControl)) + turn - (xControl * Math.abs(xControl));
            motorPowers[1] = (yControl * Math.abs(yControl)) + turn + (xControl * Math.abs(xControl));
            motorPowers[2] = (yControl * Math.abs(yControl)) - turn + (xControl * Math.abs(xControl));
            motorPowers[3] = (yControl * Math.abs(yControl)) - turn - (xControl * Math.abs(xControl));
            
            /*motorPowers[0] = Range.clip((yControl * Math.abs(yControl)) + turn - (xControl * Math.abs(xControl)), -1.0, 1.0);
            motorPowers[1] = Range.clip((yControl * Math.abs(yControl)) + turn + (xControl * Math.abs(xControl)), -1.0, 1.0);
            motorPowers[2] = Range.clip((yControl * Math.abs(yControl)) - turn + (xControl * Math.abs(xControl)), -1.0, 1.0);
            motorPowers[3] = Range.clip((yControl * Math.abs(yControl)) - turn - (xControl * Math.abs(xControl)), -1.0, 1.0);
            */
/*
            motorPowers[0] = Range.clip(drive + turn - strafe, -1.0, 1.0);
            motorPowers[1] = Range.clip(drive + turn + strafe, -1.0, 1.0);
            motorPowers[2] = Range.clip(drive - turn + strafe, -1.0, 1.0);
            motorPowers[3] = Range.clip(drive - turn - strafe, -1.0, 1.0);
*/
            // Send calculated power to wheels
            motorRef[0].setPower(motorPowers[0]);
            motorRef[1].setPower(motorPowers[1]);
            motorRef[2].setPower(motorPowers[2]);
            motorRef[3].setPower(motorPowers[3]);
            
            //LIFT
           if (gamepad2.right_trigger < 0.5){
            if (gamepad2.dpad_down) {
              lift.setTargetPosition(liftLow);
              lift.setPower(1);
            } else if (gamepad2.dpad_left) {
              lift.setTargetPosition(liftMed);
              lift.setPower(1);
            } else if (gamepad2.dpad_up) {
              lift.setTargetPosition(liftHigh);
              lift.setPower(1);
            } else if (gamepad2.dpad_right) {
              lift.setTargetPosition(liftZero);
              lift.setPower(1);
            }
           }
          
           else if (gamepad2.right_trigger > 0.5){
               if (gamepad2.dpad_right){
                   lift.setTargetPosition(liftStack);
                   lift.setPower(1);
               }
               else if (gamepad2.dpad_down){
                   lift.setTargetPosition(lift.getTargetPosition()-5);
                   lift.setPower(0.45);
               }
               else if (gamepad2.dpad_up){
                   lift.setTargetPosition(lift.getTargetPosition()+5);
                   lift.setPower(0.45);
               }
               else if (gamepad2.dpad_left){
                   stackModifier = stackIncrement * 20;
                   lift.setTargetPosition(liftStack - stackModifier);
                   lift.setPower(1);
                   stackIncrement++;
                   if (stackIncrement > 5){
                       stackIncrement = 0;
                   }
               }
           }
           
           //CLAW
           if (gamepad2.left_bumper || gamepad1.b) {
              claw.setPosition(clawOpen);
           } else if (gamepad2.right_bumper) {
              claw.setPosition(clawClose);
           } else if (gamepad1.a) {
               claw.setPosition(clawBeacon);
           }
           
           //ARM
           if (gamepad2.right_stick_y > 0.8 && gamepad2.right_trigger < 0.8) {
              arm.setPosition(armFront);
           } else if (gamepad2.right_stick_x > 0.8 && gamepad2.right_trigger < 0.8) {
              arm.setPosition(armSide);
           } else if (gamepad2.right_stick_y < -0.8 && gamepad2.right_trigger < 0.8) {
              arm.setPosition(armBack);
           }
           
           if (gamepad2.right_trigger > 0.8) {
              arm.setPosition(arm.getPosition() + gamepad2.left_stick_y * 0.001);
           }
           
           /*
           if (!gamepad2.start && gamepad2.b && !liftMode) {
               if (liftRunMode == DcMotor.RunMode.RUN_TO_POSITION) {
                   lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               } else {
                   lift.setTargetPosition(0);
                   lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               }
               liftMode = true;
           } else if (!gamepad2.b) {
               liftMode = false;
           }*/
           
           if (liftRunMode == DcMotor.RunMode.RUN_USING_ENCODER) {
               if (lift.getCurrentPosition() < liftHigh && lift.getCurrentPosition() > liftZero) {
                   lift.setPower(gamepad2.left_stick_y);
               }
           }
           
           //AUTOMATION
           if (gamepad2.a) {
              arm.setPosition(armFront);
           }

           telemetry.addData("Status", "Run Time: " + runtime.toString());
           telemetry.addData("Lift Position", lift.getCurrentPosition());
           telemetry.update();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: $runtime");
            telemetry.addData("Motors", "LF (%.2f), RF (%.2f), LB (%.2f), RB (%.2f)", motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
            telemetry.addData("input", gamepadDegree);
            
            
            telemetry.update();
        }
        
        
    }
    
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    
}

