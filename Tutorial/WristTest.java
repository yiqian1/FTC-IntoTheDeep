package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class WristTest extends LinearOpMode {
    private DcMotor  armMotor;
    private Blinker control_Hub;
    private DcMotor leftBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor rightFrontMotor;
    private Servo wrist;
    private CRServo intake;
    IMU imu; // inertial measurement unit
    final double ARM_TICKS_PER_DEGREE = -19.7924893140647; //exact fraction is (194481/9826)

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.6;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.5;
    final double WRIST_FOLDED_OUT  = 1;
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;

    public void encoderDrive(double speed, double inches) {
        double TICKS_PER_INCH = 25.89;
        int target = (int)(inches * TICKS_PER_INCH);
            
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        leftFrontMotor.setPower(-speed);
        leftBackMotor.setPower(-speed);
        rightFrontMotor.setPower(speed);
        rightBackMotor.setPower(speed);
            
        while (opModeIsActive() && rightFrontMotor.getCurrentPosition() < target);
            
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    double getHeading() {
        double ticksPerDegree = 4.5;
        return leftFrontMotor.getCurrentPosition() / (ticksPerDegree);
    }
    
    public void encoderTurn(int deg, int speed) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        leftFrontMotor.setPower(speed * (deg/Math.abs(deg)));
        leftBackMotor.setPower(speed * (deg/Math.abs(deg)));
        rightFrontMotor.setPower(speed * (deg/Math.abs(deg)));
        rightBackMotor.setPower(speed * (deg/Math.abs(deg)));
        
        while (Math.abs(deg) > Math.abs(getHeading()));
        
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        leftBackMotor = hardwareMap.get(DcMotor.class, "Left Back Motor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left Front Motor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "Right Back Motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "Right Front Motor");
        armMotor   = hardwareMap.get(DcMotor.class, "Arm"); //the arm motor
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
        imu = hardwareMap.get(IMU.class, "imu");
        
        /*intake.setPower(INTAKE_OFF);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   */
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        //armPosition = 150 * ARM_TICKS_PER_DEGREE;
        //armMotor.setTargetPosition((int) (armPosition));
        //((DcMotorEx) armMotor).setVelocity(2100);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setPower(1.0);
        //intake.setPower(INTAKE_COLLECT);
        while (opModeIsActive() && armMotor.isBusy()); 
        //((DcMotorEx) armMotor).setVelocity(0);
        //armMotor.setPower(0);
        wrist.setPosition(WRIST_FOLDED_OUT);
        
        /*encoderTurn(125, 2);
        sleep(1000);
        encoderDrive(2, 5.1);*/
        /*
        encoderDrive(2, 18);
        encoderTurn(90, 2);
        
        encoderDrive(2, 25);
        encoderTurn(-90, 2);

        encoderDrive(2, 18);
        */
        /*sleep(200);
        intake.setPower(INTAKE_DEPOSIT);
        sleep(1000);
        armPosition = ARM_COLLAPSED_INTO_ROBOT;
        armMotor.setTargetPosition((int)armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0); // Adjust power as needed
        while (opModeIsActive() && armMotor.isBusy()); // Wait until the arm has collapsed
        armMotor.setPower(0);
        
        encoderTurn(-240,2);
        sleep(100);
        encoderDrive(2,45);
        sleep(300);
        encoderTurn(-130,2);
        sleep(300);
        encoderDrive(2, 15);
        /*
        /*
        telemetry.addData("armTarget: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        telemetry.update();*/
    }
}
