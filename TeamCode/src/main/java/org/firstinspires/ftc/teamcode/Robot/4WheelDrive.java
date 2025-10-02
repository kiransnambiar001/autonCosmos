package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // For linear OpModes
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; // For TeleOp OpModes
import com.qualcomm.robotcore.hardware.DcMotor; // For DC motors
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name="4Wheel Drive OpMode", group="LinearOpMode")
public class FourWheelDrive  extends LinearOpMode {


    // Create hardware object
    Hardware robot = new Hardware();


    @Override
    public void runOpMode() throws InterruptedException {

        Hardware robot = new Hardware();

        // update telemetry to show INITIALIZED status
        telemetry.addData("Status", "INITIALIZED");
        telemetry.update();

        // wait for user to press start button
        waitForStart();

        // start OpMode loop
        while (opModeIsActive()) {
            // get data from controller
            double axial = -gamepad1.left_stick_y; // forward/backward driving
            double lateral = gamepad1.left_stick_x; // strafing
            double yaw = gamepad1.right_stick_x; // turning

            // Calculate motor powers
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // limit max motor power
            frontLeftPower = Math.max(-1.0,Math.min(1.0, frontLeftPower));
            frontRightPower = Math.max(-1.0,Math.min(1.0, frontRightPower));
            backLeftPower = Math.max(-1.0,Math.min(1.0, backLeftPower));
            backRightPower = Math.max(-1.0,Math.min(1.0, backRightPower));

            // set motor power based on values
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            /* Telemetry
            telemetry.addData("Status", "RUNNING");
            telemetry.addData("FrontLeft Motor Power", frontLeftPower);
            telemetry.addData("FrontRight Motor Power", frontRightPower);
            telemetry.addData("BackLeft Motor Power", backLeftPower);
            telemetry.addData("BackRight Motor Power", backRightPower);
            telemetry.update();*/
        }
    }
}