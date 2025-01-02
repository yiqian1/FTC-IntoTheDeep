package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver Station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous
//@Disabled
public class AutonomousRed extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor  armMotor;
    private Blinker control_Hub;
    private DcMotor leftBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor slideMotorRight;
    private DcMotor slideMotorLeft;
    private Servo leftArm, rightArm;
    private Servo claw; 
    private CRServo intake;
    IMU imu = null; // inertial measurement unit
    final double ARM_TICKS_PER_DEGREE = -19.7924893140647; //exact fraction is (194481/9826)

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    
    
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;   // eg: GoBILDA 435 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double SLIDE_COUNTS_PER_MOTOR_REV = 537.7; // Gobilda 312 RPM Motor
    static final double SLIDE_COUNTS_PER_INCH = SLIDE_COUNTS_PER_MOTOR_REV/(1.5*3.1415);

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    // Claw positions
     final double CLAW_CLOSE = 0.0;
     final double CLAW_OPEN = 1.0;
     
     // Arm positions
     final double ARM_INITIAL = 0.0; 
     final double ARM_COLLECT = 0.55;
     final double ARM_ASCENT = 0.17;
     final double ARM_SCORE_SPECIMEN = 0.2;
     final double ARM_SCORE_BASKETS = 0.3;
     
     // Slides positions
     final double SLIDES_INITIAL = 0.0;
     final double SLIDES_MEDIUM = 0.0;
     final double SLIDES_SCORE_SPECIMEN = 11 * SLIDE_COUNTS_PER_INCH;
     final double SLIDES_HIGH_BASKET = 30 * SLIDE_COUNTS_PER_INCH;
     final double SLIDES_LOW_BASKET = 15 * SLIDE_COUNTS_PER_INCH;
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double slidePosition = 0;
    
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;
    
    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.5;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.7;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 1.0;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftBackMotor = hardwareMap.get(DcMotor.class, "Left Back Motor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left Front Motor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "Right Back Motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "Right Front Motor");
        //Initialize the slide variables
        slideMotorRight = hardwareMap.get(DcMotor.class, "slideMotorRight");
        slideMotorLeft = hardwareMap.get(DcMotor.class, "slideMotorLeft");
        
        slideMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        slideMotorRight.setDirection(DcMotor.Direction.FORWARD);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset the encoders of the slide motors
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm.setPosition(ARM_INITIAL);
        leftArm.setPosition(1-ARM_INITIAL);
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_CLOSE);
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
        
        slideMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        slideMotorRight.setTargetPosition(0);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setTargetPosition(0);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw();
        
        // // Step through each leg of the path,
        // // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        // //          holdHeading() is used after turns to let the heading stabilize
        // //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        waitForStart();
        claw.setPosition(CLAW_CLOSE);
        
        // slidePosition = 40.0;
        // slideMotorLeft.setTargetPosition((int)(15*SLIDE_COUNTS_PER_INCH));
        // slideMotorRight.setTargetPosition((int)(15*SLIDE_COUNTS_PER_INCH));
        // slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // slideMotorRight.setPower(0.8);
        // slideMotorLeft.setPower(0.8); // Adjust power as needed
        // while (opModeIsActive() && slideMotorRight.isBusy()); // Wait until the arm has collapsed
        // slideMotorRight.setPower(0.2);
        //  slideMotorLeft.setPower(0.2);

        telemetry.addData("lift 20 inches", "Complete");
        telemetry.update();
        /*
        slideMotorRight.setPower(0);
        slideMotorLeft.setPower(0);
        rightArm.setPosition(0.55);
        leftArm.setPosition(1-0.55);
        sleep(1500);
        claw.setPosition(CLAW_CLOSE);
        sleep(1000);
        rightArm.setPosition(0);
        leftArm.setPosition(1);
        */
        /*
        driveStraight(DRIVE_SPEED, -10, 0);
        sleep(300);
        slideMotorLeft.setTargetPosition((int)(0));
        slideMotorRight.setTargetPosition((int)(0));
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setPower(0.8);
        slideMotorLeft.setPower(0.8); // Adjust power as needed
        while (opModeIsActive() && slideMotorRight.isBusy()); // Wait until the arm has collapsed
        */
        
        driveStraight(DRIVE_SPEED, -20, 0.0);
        turnToHeading(TURN_SPEED, -45);
        slideToPosition(SLIDES_HIGH_BASKET);
        driveStraight(DRIVE_SPEED, 20, -45.0);
        armToPosition(ARM_SCORE_BASKETS);
        sleep(400);
        claw.setPosition(CLAW_OPEN);
        sleep(400);
        armToPosition(ARM_INITIAL);
        
        // // Move backward
        driveStraight(DRIVE_SPEED, -23, -45);
        resetRobot();
        turnToHeading(TURN_SPEED, 0);
        driveStraight(DRIVE_SPEED, -27, 0);
        
        // Push the 1st sample
        turnToHeading(TURN_SPEED, -30);
        driveStraight(0.4, 46, -30);
        
        // // Move backward
        driveStraight(DRIVE_SPEED, -32, -25);
        turnToHeading(TURN_SPEED, 0);
        driveStraight(DRIVE_SPEED, -8, 0);

        // // Push the 2nd sample
        turnToHeading(TURN_SPEED, -30);
        driveStraight(0.3, 18, -30);
        driveStraight(0.3, 28, -13);
        //Move backward
        driveStraight(DRIVE_SPEED, -45, -10);
        turnToHeading(TURN_SPEED, 0);
        // Move sideways
        goSideways(0.4);
        // //Push in the 3rd sample
        driveStraight(0.3, 50, 0); 
        
        // wrist.setPosition(WRIST_FOLDED_IN);
        // armPosition = ARM_COLLAPSED_INTO_ROBOT;
        // armMotor.setTargetPosition((int)armPosition);
        // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // armMotor.setPower(1.0); // Adjust power as needed
        // while (opModeIsActive() && armMotor.isBusy()); // Wait until the arm has collapsed
        // armMotor.setPower(0);
        resetRobot();
        
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
        
    }
    
    public void goSideways(double maxDriveSpeed){
        double speed = Math.abs(maxDriveSpeed);
     
        leftBackMotor.setPower(-speed);
        leftFrontMotor.setPower(speed);
        rightBackMotor.setPower(speed);
        rightFrontMotor.setPower(-speed);

        sleep(800);
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     }

    // /*
    //  * ====================================================================================================
    //  * Driving "Helper" functions are below this line.
    //  * These provide the high and low level methods that handle driving straight and turning.
    //  * ====================================================================================================
    //  */

    // // **********  HIGH Level driving functions.  ********************

    // /**
    //  *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    //  *  Move will stop if either of these conditions occur:
    //  *  1) Move gets to the desired position
    //  *  2) Driver stops the OpMode running.
    //  *
    //  * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    //  * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    //  * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    //  *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    //  *                   If a relative angle is required, add/subtract from the current robotHeading.
    //  */
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

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
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

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
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

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
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

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
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

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
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

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    
    public void slideToPosition(double slidePosition) {
        slideMotorLeft.setTargetPosition((int)slidePosition);
        slideMotorRight.setTargetPosition((int)slidePosition);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setPower(0.8);
        slideMotorLeft.setPower(0.8); // Adjust power as needed
        while (opModeIsActive() && slideMotorRight.isBusy());
        slideMotorRight.setPower(0.2);
        slideMotorLeft.setPower(0.2);
    }
    
    public void armToPosition(double armPosition) {
        rightArm.setPosition(armPosition);
        leftArm.setPosition(1-armPosition);
    }
    
    public void resetRobot(){
        armToPosition(ARM_INITIAL);
        slideToPosition(SLIDES_INITIAL);
    }
}
