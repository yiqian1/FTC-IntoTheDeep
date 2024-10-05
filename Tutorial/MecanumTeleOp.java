package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("Left Front Motor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("Left Back Motor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("Right Front Motor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("Right Back Motor");
        telemetry.addData("Intialize",1);
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Remove if needed
        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Remove if needed

        waitForStart();
        telemetry.addData("Start",1);
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double drive /*Front and Back*/ = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double strafe /*Left to Right*/ = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double twist /*Turning*/ = -gamepad1.right_stick_x; // This is the RIGHT stick, indpendent of drive and strafe
            telemetry.addData("drive", drive);
            telemetry.addData("strafe", strafe);
            telemetry.addData("twist", twist);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(twist), 1);
            double frontLeftPower = (drive + strafe + twist) / denominator; // ++
            double backLeftPower = (drive - strafe + twist) / denominator; //-+
            double frontRightPower = (drive - strafe - twist) / denominator; //--
            double backRightPower = (drive + strafe - twist) / denominator; // +-

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);   
            backRightMotor.setPower(backRightPower);
        }
    }
}
