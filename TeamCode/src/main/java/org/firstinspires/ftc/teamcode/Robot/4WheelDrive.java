package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // For linear OpModes
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; // For TeleOp OpModes
import com.qualcomm.robotcore.hardware.DcMotor; // For DC motors
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name="4Wheel Drive OpMode", group="LinearOpMode")
public class FourWheelDrive  extends LinearOpMode {


    // Create hardware object
    Hardware robotHardware = new Hardware();


    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware.initialize(hardwareMap);

        // update telemetry to show INITIALIZED status
        telemetry.addData("Status", "INITIALIZED");
        telemetry.update();

        // wait for user to press start button
        waitForStart();

        float speedMultiplier = 1.0f;
        boolean button1prevState = false;
        boolean slowMode = false;

        // start OpMode loop
        while (opModeIsActive()) {
            // get data from controller
            double axial = -gamepad1.left_stick_y; // forward/backward driving
            double lateral = gamepad1.left_stick_x; // strafing
            double yaw = gamepad1.right_stick_x; // turning
            boolean button1state = gamepad1.b; // slow mode


            if (button1state == true && button1prevState == false) {
                slowMode = !slowMode;
                speedMultiplier = slowMode ? 0.3 : 1.0;
            }
            button1prevState = button1state;

            // Calculate motor powers
            double frontLeftPower = (axial + lateral + yaw)*speedMultiplier;
            double frontRightPower = (axial - lateral - yaw)*speedMultiplier;
            double backLeftPower = (axial - lateral + yaw)*speedMultiplier;
            double backRightPower = (axial + lateral - yaw)*speedMultiplier;

            // limit max motor power
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            // If the calculated max power is greater than 1.0, scale all powers down proportionally.
            if (maxPower > 1.0) {
                frontLeftPower  /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower   /= maxPower;
                backRightPower  /= maxPower;
            }

            // set motor power based on values
            robotHardware.frontLeft.setPower(frontLeftPower);
            robotHardware.frontRight.setPower(frontRightPower);
            robotHardware.backLeft.setPower(backLeftPower);
            robotHardware.backRight.setPower(backRightPower);

            /* Telemetry
            telemetry.addData("Status", "RUNNING");
            telemetry.addData("FrontLeft Motor Power", frontLeftPower);
            telemetry.addData("FrontRight Motor Power", frontRightPower);
            telemetry.addData("BackLeft Motor Power", backLeftPower);
            telemetry.addData("BackRight Motor Power", backRightPower);
            telemetry.update();*/
            //Im stupid
        }
    }
}