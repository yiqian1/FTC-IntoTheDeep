package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous
//@Disabled
public class AA2_SlideSpeciAuto extends LinearOpMode {

    /* Declare OpMode members. */
     public DcMotor  armMotor   = null; //the arm motor
     public CRServo  wrist      = null; //the active intake servo
     public Servo    intake     = null; //the wrist servo
    
     // Declaring motors and servos
     private static DcMotor leftFrontMotor;
     private static DcMotor leftBackMotor;
     private static DcMotor rightFrontMotor;
     private static DcMotor rightBackMotor;
     private static DcMotor slideMotorLeft;
     private static DcMotor slideMotorRight;
     private static Servo leftArm, rightArm;
     private static Servo claw; 
     IMU imu = null;
     private static GoBildaPinpointDriver odo;
     
     static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;   // eg: GoBILDA 435 RPM Yellow Jacket
     static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
     static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
     static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double SLIDE_COUNTS_PER_MOTOR_REV = 537.7; // Gobilda 312 RPM Motor
    static final double SLIDE_COUNTS_PER_INCH = SLIDE_COUNTS_PER_MOTOR_REV/(1.5*3.1415);
     
     public static double speed = 1.5; // Speed
     
     // Claw positions
     final double CLAW_CLOSE = 1;
     final double CLAW_OPEN = 0.5;
     
     // Arm positions
     final double ARM_INITIAL = 0.0; 
     final double ARM_COLLECT = 0.55;
     final double ARM_ASCENT = 0.17;
     final double ARM_SCORE_SPECIMEN = 0.2;
     final double ARM_SCORE_BASKETS = 0.3;
     
     // Slides positions
     final double SLIDES_INITIAL = 0.0;
     final double SLIDES_MEDIUM = 0.0;
     final double SLIDES_SCORE_SPECIMEN = 7.5 * SLIDE_COUNTS_PER_INCH;
     final double SLIDES_HIGH_BASKET = 30 * SLIDE_COUNTS_PER_INCH;
     final double SLIDES_LOW_BASKET = 15 * SLIDE_COUNTS_PER_INCH;
     
    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    public final double     DRIVE_SPEED             = 0.8;     // Max driving speed for better distance accuracy.
    public final double     TURN_SPEED              = 0.7;     // Max turn speed to limit turn rate.
    public final double     HEADING_THRESHOLD       = 1.0;    // How close must the heading get to the target before moving to next step.
    public final double     SLIDES_KP               = 0.001; 
    public final double     SLIDES_BASE_POWER       = 0.2;
    
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.
    static final double     P_STRAFE_GAIN          = 0.01; // up to change
    
    static final double DRIVE_DRIVE = 0.03;
    static final double DRIVE_STRAFE = 0.01;
    static final double DRIVE_TURN = 0.008;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;
    private double  headingError  = 0;
    
    @Override
    public void runOpMode() {
        initRobot();
        
        claw.setPosition(CLAW_CLOSE);
        
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        claw.setPosition(CLAW_CLOSE);
        
        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        // Set the encoders for closed loop speed control, and reset the heading.
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        imu.resetYaw();
        
        claw.setPosition(CLAW_CLOSE);
        
        waitForStart();
        
        claw.setPosition(CLAW_CLOSE);
        sleep(100);
        // Go to submersible
        slideToPosition(SLIDES_SCORE_SPECIMEN);
        armToPosition(ARM_SCORE_SPECIMEN, 500);
        
        // Score the first speci
        drive(28, DRIVE_SPEED);
        
        slideToPosition(2.5*SLIDE_COUNTS_PER_INCH);
        slideMotorRight.setPower(0);
        slideMotorLeft.setPower(0);
        claw.setPosition(CLAW_OPEN);
        
        // Go back
        drive(-25, DRIVE_SPEED);
        // Slides to initial position
        slideToPosition(0*SLIDE_COUNTS_PER_INCH);
        slideMotorRight.setPower(0);
        slideMotorLeft.setPower(0);
        turn(-90, TURN_SPEED);
        drive(18, DRIVE_SPEED);
        // pick up 2nd specimen
        armToPosition(ARM_COLLECT, 500);
        claw.setPosition(CLAW_CLOSE);
        sleep(200);
        armToPosition(ARM_INITIAL, 500);
        drive(-23, DRIVE_SPEED);
        //Turn to submersible
        turn(90, TURN_SPEED);

        slideToPosition(SLIDES_SCORE_SPECIMEN);
        armToPosition(ARM_SCORE_SPECIMEN, 500);
        drive(24.5, TURN_SPEED);
        //Hang 2nd specimen
        slideToPosition(0*SLIDE_COUNTS_PER_INCH);
        slideMotorRight.setPower(0);
        slideMotorLeft.setPower(0);
        claw.setPosition(CLAW_OPEN);
        drive(-10, DRIVE_SPEED); 
        // Move to samples
        strafe(-31, DRIVE_SPEED);
        drive(38, DRIVE_SPEED);
        turn(177, TURN_SPEED);
        
        odo.update();
            Pose2D pos = odo.getPosition();
            telemetry.addData("after reset X:", pos.getX(DistanceUnit.INCH));
            telemetry.addData("after reset Y:", pos.getY(DistanceUnit.INCH));
            telemetry.addData("after reset H:", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        
        strafe(11, DRIVE_SPEED);
        // Push the 1st sample
        drive(50, DRIVE_SPEED);

        /*
        sleep(200);
        drive(-50, DRIVE_SPEED);
        // Push the 2nd sample
        strafe(11, DRIVE_SPEED);
        drive(50, DRIVE_SPEED);
        sleep(200);
        drive(-50, DRIVE_SPEED);
        // Push the 3rd sample
        strafe(7, DRIVE_SPEED);
        //sleep(200);
        drive(51, DRIVE_SPEED-0.2);*/
        resetRobot();

    }
    
    /* FUNCTIONS */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            int leftBackTarget = leftBackMotor.getCurrentPosition() + moveCounts;
            int leftFrontTarget = leftFrontMotor.getCurrentPosition() + moveCounts;
            int rightBackTarget = rightBackMotor.getCurrentPosition() + moveCounts;
            int rightFrontTarget = rightFrontMotor.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftBackMotor.setTargetPosition(leftBackTarget);
            leftFrontMotor.setTargetPosition(leftFrontTarget);
            rightBackMotor.setTargetPosition(rightBackTarget);
            rightFrontMotor.setTargetPosition(rightFrontTarget);

            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftFrontMotor.isBusy() && rightFrontMotor.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    public void drive(double distance, double speed) {
        if (opModeIsActive()) {
        odo.update();
        Pose2D pos = odo.getPosition();
        
        distance = pos.getX(DistanceUnit.INCH) + distance;
        double currentStrafe = pos.getY(DistanceUnit.INCH);
        double currentHeading = pos.getHeading(AngleUnit.DEGREES);
        double driveError = distance - pos.getX(DistanceUnit.INCH);
        boolean isDone = Math.abs(driveError) < 0.5;
        speed = Math.abs(speed);
        
        while (opModeIsActive() && !isDone) {
            odo.update();
            pos = odo.getPosition();
            telemetry.addData("driveX:", pos.getX(DistanceUnit.INCH));
            telemetry.addData("driveY:", pos.getY(DistanceUnit.INCH));
            telemetry.addData("driveH:", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            driveError = distance - pos.getX(DistanceUnit.INCH);
            double driveCorrection = driveError * DRIVE_DRIVE;
                 
            double strafeError = currentStrafe - pos.getY(DistanceUnit.INCH);
            double strafeCorrection = strafeError * DRIVE_STRAFE;
                 
            double turnError = currentHeading - pos.getHeading(AngleUnit.DEGREES);
            double turnCorrection = turnError * DRIVE_TURN;
            
            driveCorrection = Range.clip(driveCorrection, -speed, speed);
            strafeCorrection = Range.clip(strafeCorrection, -speed, speed);
            moveRobot(driveCorrection, -strafeCorrection, -turnCorrection);
           
            isDone = Math.abs(driveError) < 0.5;
        }
        moveRobot(0, 0, 0);
        //sleep(180);
        }
    }
    
    public void strafe(double distance, double speed) {
        if (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            double currentDrive = pos.getX(DistanceUnit.INCH);
            double currentHeading = pos.getHeading(AngleUnit.DEGREES);
            double targetDistance = distance + pos.getY(DistanceUnit.INCH);
            boolean isDone = Math.abs(distance) < 0.5;
            speed = Math.abs(speed);
            ElapsedTime holdTimer = new ElapsedTime();
            while (opModeIsActive() && !isDone) {
                odo.update();
                pos = odo.getPosition();
                double driveError = currentDrive - pos.getX(DistanceUnit.INCH);
                double driveCorrection = driveError * DRIVE_STRAFE;
                 
                double strafeError = targetDistance - pos.getY(DistanceUnit.INCH);
                double strafeCorrection = strafeError * DRIVE_DRIVE;
                 
                double turnError = currentHeading - pos.getHeading(AngleUnit.DEGREES);
                double turnCorrection = turnError * DRIVE_TURN;
            
                strafeCorrection = Range.clip(strafeCorrection, -speed, speed);
                telemetry.addData("targetX:", currentDrive);
                telemetry.addData("targetY:", targetDistance);            
                telemetry.addData("strafe X:", pos.getX(DistanceUnit.INCH));
                telemetry.addData("strafe Y:", pos.getY(DistanceUnit.INCH));
                telemetry.addData("strafe H:",pos.getHeading(AngleUnit.DEGREES));
                telemetry.addData("drive:",driveCorrection);
                telemetry.addData("strafe:",-strafeCorrection);
                telemetry.addData("turn:",-turnCorrection);
                telemetry.update();
                moveRobot(driveCorrection, -strafeCorrection, -turnCorrection);
                isDone = Math.abs(strafeError) < 0.5;
                if (holdTimer.time()>2 && Math.abs(strafeError)<=1)
                  isDone = true;
            }
            moveRobot(0, 0, 0);
            sleep(100);
        }
    }
    
    public void turn(double angle, double speed){
        speed = Math.abs(speed);
        if (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            double currentH = pos.getHeading(AngleUnit.DEGREES);
            telemetry.addData("H:", currentH-angle);
            telemetry.update();
            while(opModeIsActive() && Math.abs(angle - currentH)>2){
                odo.update();
                pos = odo.getPosition();
                currentH = pos.getHeading(AngleUnit.DEGREES);
                double headingError = angle - currentH;
                double headingCorrection = headingError * 0.02;
                headingCorrection = Range.clip(headingCorrection, -speed, speed);
                telemetry.addData("H:", currentH);
                telemetry.addData("hc:", headingCorrection);
                telemetry.update();
                moveRobot(0, 0, -headingCorrection);
            }
            moveRobot(0, 0, 0);
            sleep(500); //sleep to make sure that the reset works
            odo.resetPosAndIMU();
            odo.update();
            while (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY){
            sleep(100);
            odo.update();
        }
        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }
    
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftBackMotor.setPower(leftSpeed);
        leftFrontMotor.setPower(leftSpeed);
        rightBackMotor.setPower(rightSpeed);
        rightFrontMotor.setPower(rightSpeed);
    }
    
    public void moveRobot(double drive, double strafe, double yaw) {
        double lF = drive + strafe + yaw;
        double rF = drive - strafe - yaw;
        double lB = drive - strafe + yaw;
        double rB = drive + strafe - yaw;
             
        double max = Math.max(Math.abs(lF), Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));
             
        if (max > 1.0)  {
            lF /= max;
            rF /= max;
            lB /= max;
            rB /= max;
        }
             
        //send power to the motors
        leftFrontMotor.setPower(lF);
        rightFrontMotor.setPower(rF);
        leftBackMotor.setPower(lB);
        rightBackMotor.setPower(rB);
    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftFrontMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    
    public void initRobot(){
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        odo.setOffsets(-63.5, 177.8);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        // Declare our motors
        leftFrontMotor = hardwareMap.dcMotor.get("Left Front Motor");
        leftBackMotor = hardwareMap.dcMotor.get("Left Back Motor");
        rightFrontMotor = hardwareMap.dcMotor.get("Right Front Motor");
        rightBackMotor = hardwareMap.dcMotor.get("Right Back Motor");
         
        // Reverse the left side motors.
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE); 
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         
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
        claw.setPosition(CLAW_CLOSE);
         
        slideMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        slideMotorRight.setTargetPosition(0);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setTargetPosition(0);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         
        telemetry.addData("Intialize",1);
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
                slideMotorRight.setPower(correction + SLIDES_BASE_POWER);
                slideMotorLeft.setPower(correction + SLIDES_BASE_POWER);
            }
        }
        slideMotorRight.setPower(0.2);
        slideMotorLeft.setPower(0.2);
    }
    
    public void armToPosition(double armPosition, int waitTime) {
        rightArm.setPosition(armPosition);
        leftArm.setPosition(1-armPosition);
        sleep(waitTime);
    }
    
    public void resetRobot(){
        armToPosition(ARM_INITIAL, 200);
        slideToPosition(SLIDES_INITIAL);
    }
    
    public void goSideways(double maxDriveSpeed, int time){
         double speed = maxDriveSpeed;
     
         leftBackMotor.setPower(speed);
         leftFrontMotor.setPower(-speed);
         rightBackMotor.setPower(-speed);
         rightFrontMotor.setPower(speed);

         sleep(time);
         leftBackMotor.setPower(0);
         leftFrontMotor.setPower(0);
         rightBackMotor.setPower(0);
         rightFrontMotor.setPower(0);
         leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     }
}
