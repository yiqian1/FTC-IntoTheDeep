package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.annotation.Target;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SecondTeleOp extends LinearOpMode {
     /* Declare OpMode members. */
     public DcMotor  armMotor    = null; //the arm motor
     public CRServo  wrist      = null; //the active intake servo
     public Servo    intake       = null; //the wrist servo
    
     // Declaring motors and servos
     private static DcMotor frontLeftMotor;
     private static DcMotor backLeftMotor;
     private static DcMotor frontRightMotor;
     private static DcMotor backRightMotor;
     private static DcMotor slideMotorLeft;
     private static DcMotor slideMotorRight;
     private static Servo leftArm, rightArm;
     private static Servo claw; 
     
     static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;   // eg: GoBILDA 435 RPM Yellow Jacket
     static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
     static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
     static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double SLIDE_COUNTS_PER_MOTOR_REV = 537.7; // Gobilda 312 RPM Motor
    static final double SLIDE_COUNTS_PER_INCH = SLIDE_COUNTS_PER_MOTOR_REV/(1.5*3.1415);
     
     public static double speed = 1.5; // Speed
     
     // Claw positions
     final double CLAW_CLOSE = 0.0;
     final double CLAW_OPEN = 1.0;
     
     // Arm positions
     final double ARM_INITIAL = 0.0; 
     final double ARM_COLLECT = 0.55;
     final double ARM_ASCENT = 0.17;
     final double ARM_SCORE_SPECIMEN = 0.2;
     final double ARM_SCORE_BASKETS = 0.3;
     final double ARM_CLEAR_BARRIER = 0.4;
     
     // Slides positions
     final double SLIDES_INITIAL = 0.0;
     final double SLIDES_MEDIUM = 0.0;
     final double SLIDES_SCORE_SPECIMEN = 7.5 * SLIDE_COUNTS_PER_INCH;
     final double SLIDES_HIGH_BASKET = 30 * SLIDE_COUNTS_PER_INCH;
     final double SLIDES_LOW_BASKET = 15 * SLIDE_COUNTS_PER_INCH;
     final double SLIDES_KP = 0.001; 

     @Override
     public void runOpMode() throws InterruptedException {
        double slidePosition = SLIDES_INITIAL;
        double armPosition = ARM_INITIAL;
        initRobot();
        
         waitForStart();
         telemetry.addData("Start",1);
        
         if (isStopRequested()) return;
       
         while (opModeIsActive()) {

            chassisControl();
            armControl();
            slideControl();
            
//             armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger)); 
//             if (!(gamepad2.right_trigger > 0.1) && !(gamepad2.left_trigger > 0.1)){
//                 armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));
//             }
            
                 if(gamepad2.y){
                     /* This is the correct height to score HIGH BASKET */
                     slideToPosition(SLIDES_HIGH_BASKET);
                     armToPosition(ARM_SCORE_BASKETS);
                 }

                 else if (gamepad2.x){
                         /* This is the correct height to score the sample in the LOW BASKET */
                         slideToPosition(SLIDES_LOW_BASKET);
                         armToPosition(ARM_SCORE_BASKETS);
                 }
    
                 else if (gamepad2.b){
                         /* This is the correct height to ASCEND */
                         armToPosition(ARM_CLEAR_BARRIER);
                 }
    
                 else if (gamepad2.a){
                         /* This is the correct height to COLLECT */
                         armToPosition(ARM_COLLECT);
                         slideToPosition(SLIDES_INITIAL);
                 }
                 
                 if (gamepad2.left_bumper){
                         claw.setPosition(CLAW_OPEN);
                 }
                 else if (gamepad2.right_bumper){
                        claw.setPosition(CLAW_CLOSE);
                 }
                
                else if (gamepad2.dpad_up){
                        //This sets the arm to hook specimen 
                        slideToPosition(SLIDES_SCORE_SPECIMEN);
                        armToPosition(ARM_SCORE_SPECIMEN);
                 }
                 
                else if (gamepad2.dpad_left){
                    slideToPosition(SLIDES_INITIAL);
                    claw.setPosition(CLAW_OPEN);
                    //armToPosition(ARM_INITIAL);
                }
                 else if (gamepad2.dpad_down){
                         //this moves everything to original 
                         armToPosition(ARM_INITIAL);
                         slideToPosition(SLIDES_INITIAL);
                 }
                 
                 else if (gamepad2.dpad_right){
                    armToPosition(ARM_ASCENT);
                 }
                 
                 else if (gamepad1.left_trigger > 0.3){
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                 }
//               /* Here we set the target position of our arm to match the variable that was selected
//                 by the driver.
//                 We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
                
//                 armMotor.setTargetPosition((int) (armPosition+armPositionFudgeFactor));
            
//                 ((DcMotorEx) armMotor).setVelocity(1600);
//                 armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
//                 /* Check to see if our arm is over the current limit, and report via telemetry. */
//                 if (((DcMotorEx) armMotor).isOverCurrent()){
//                     telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
//                 }

//                 /* send telemetry to the driver of the arm's current position and target position */
//                 telemetry.addData("armTarget: ", armMotor.getTargetPosition());
//                 telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
//                 telemetry.update();
                
                
       }
    }
    public void initRobot(){
        // Declare our motors
        frontLeftMotor = hardwareMap.dcMotor.get("Left Front Motor");
        backLeftMotor = hardwareMap.dcMotor.get("Left Back Motor");
        frontRightMotor = hardwareMap.dcMotor.get("Right Front Motor");
        backRightMotor = hardwareMap.dcMotor.get("Right Back Motor");
         
        // Reverse the left side motors.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE); 
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         
        slideMotorRight = hardwareMap.get(DcMotor.class, "slideMotorRight");
        slideMotorLeft = hardwareMap.get(DcMotor.class, "slideMotorLeft");
        
        // Set the directions of the slide motors
        slideMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        slideMotorRight.setDirection(DcMotor.Direction.FORWARD);
         
        // Arm and claw servos
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm.setPosition(ARM_INITIAL);
        leftArm.setPosition(1-ARM_INITIAL);
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_OPEN);
         
        slideMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Makes sure the motors holds its position after running
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        slideMotorRight.setTargetPosition(0);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setTargetPosition(0);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         
        telemetry.addData("Intialize",1);
    }
    public void chassisControl(){
        // left stick: up/down to drive, left/right to strafe
        // right stick: left/right to turn
        double drive /*Front and Back*/ = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double strafe /*Left to Right*/ = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double twist /*Turning*/ = gamepad1.right_stick_x; // This is the RIGHT stick, indpendent of drive and strafe

        if (Math.abs(drive) < 0.2)
            drive = 0;
            
        if (Math.abs(strafe) < 0.2)
            strafe = 0;
            
        if (Math.abs(twist) < 0.2)
            twist = 0;
            
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(twist), 2);
        double frontLeftPower = (drive + strafe + twist) / denominator;
        double frontRightPower = (drive - strafe - twist) / denominator;
        double backLeftPower = (drive - strafe + twist) / denominator;
        double backRightPower = (drive + strafe - twist) / denominator;

        frontLeftMotor.setPower(frontLeftPower*speed); 
        backLeftMotor.setPower(backLeftPower*speed); 
        frontRightMotor.setPower(frontRightPower*speed);
        backRightMotor.setPower(backRightPower*speed);
    }
    
    public void slideToPosition(double targetPosition) {
        slideMotorLeft.setTargetPosition((int)targetPosition);
        slideMotorRight.setTargetPosition((int)targetPosition);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setPower(0.8);
        slideMotorLeft.setPower(0.8); // Adjust power as needed
        while (opModeIsActive() && slideMotorRight.isBusy()){
            double currentPosition = slideMotorRight.getCurrentPosition();
            double currentError = targetPosition - currentPosition;
            double correction = currentError * SLIDES_KP; 
            if(currentError > 0){
                slideMotorRight.setPower(correction + 0.2);
                slideMotorLeft.setPower(correction + 0.2);
            }
        }
        slideMotorRight.setPower(2.0);
        slideMotorLeft.setPower(2.0);
    }
    
    public void armToPosition(double armPosition) {
        rightArm.setPosition(armPosition);
        leftArm.setPosition(1-armPosition);
    }
    
    public void armControl() {
        double armPosition = rightArm.getPosition();
        if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2){
            rightArm.setPosition(armPosition + gamepad2.right_stick_y * 0.01);
            leftArm.setPosition(1-(armPosition + gamepad2.right_stick_y * 0.01));
        }
    }
    
    public void slideControl() {
        int slidePosition = slideMotorLeft.getCurrentPosition();
        slideToPosition(slidePosition - gamepad2.left_stick_y * 100);
    }
}

