package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SlidesTest", group = "TeleOp")
public class SlidesTest extends OpMode {

    private DcMotor slideMotorRight;
    private DcMotor slideMotorLeft;

    @Override
    public void init() {
        // Initialize the slide motor (adjust the name to match your configuration)
        slideMotorRight = hardwareMap.get(DcMotor.class, "slideMotorRight");
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorLeft = hardwareMap.get(DcMotor.class, "slideMotorLeft");
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Prevent sliding when not powered
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Slide control logic using D-pad
        if (gamepad1.dpad_up) {
            // Move slides up
            slideMotorRight.setPower(-0.6);
            slideMotorLeft.setPower(0.6);// Adjust power as needed for your slide mechanism
        } else if (gamepad1.dpad_down) {
            // Move slides down
            slideMotorRight.setPower(0.6);
            slideMotorLeft.setPower(-0.6);// Adjust power as needed for your slide mechanism
        } else {
            // Stop the slides when no button is pressed
            slideMotorRight.setPower(0.0);
            slideMotorLeft.setPower(0.0);
        }

        // Telemetry for debugging
        telemetry.addData("Slide Power Right", slideMotorRight.getPower());
        telemetry.addData("Slide Power Left", slideMotorLeft.getPower());
        telemetry.update();
    }
}
