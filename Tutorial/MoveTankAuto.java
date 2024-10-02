/*
Copyright 2025 

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous

public class MoveTankAuto extends LinearOpMode {
    private Blinker control_Hub;
    private ColorSensor color;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Servo servo;
    IMU imu; // inertial measurement unit
    
    public void encoderDrive(double speed, double inches) {
        double TICKS_PER_INCH = 25.89;
        int target = (int)(inches * TICKS_PER_INCH);
            
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        leftMotor.setPower(-speed);
        rightMotor.setPower(speed);
            
        while (opModeIsActive() && rightMotor.getCurrentPosition() < target);
            
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    double getHeading() {
        double ticksPerDegree = 1.807;
        return leftMotor.getCurrentPosition() / (ticksPerDegree);
    }
    
    public void encoderTurn(int deg, int speed) {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        leftMotor.setPower(speed * (deg/Math.abs(deg)));
        rightMotor.setPower(speed * (deg/Math.abs(deg)));
        
        while (Math.abs(deg) > Math.abs(getHeading()));
        
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    
    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        color = hardwareMap.get(ColorSensor.class, "color");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        servo = hardwareMap.get(Servo.class, "servo");
        imu = hardwareMap.get(IMU.class, "imu");
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        encoderDrive(2, 25);
        encoderTurn(-90, 2);
        
        encoderDrive(2, 18);
        encoderTurn(90, 2);
        
        encoderDrive(2, 25);
        encoderTurn(-90, 2);

        encoderDrive(2, 18);

    }
}
