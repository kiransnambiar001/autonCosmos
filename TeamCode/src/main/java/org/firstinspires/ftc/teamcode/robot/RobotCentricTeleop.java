package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // For linear OpModes
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; // For TeleOp OpModes
import com.qualcomm.robotcore.hardware.IMU;


@TeleOp(name="RobotCentric TeleOp", group="LinearOpMode")

public class RobotCentricTeleop extends LinearOpMode {


    // Create hardware object
    Hardware robotHardware = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware.initialize(hardwareMap);

        // Init hardwareMaps for each motor


        float speedMultiplier = 1.0f;
        boolean slowMode = false;

        // update telemetry to show INITIALIZED status
        telemetry.addData("Status", "INITIALIZED");
        telemetry.update();

        // wait for user to press start button
        waitForStart();

        boolean button1PrevState = false;

        // start OpMode loop
        while (opModeIsActive()) {

            // get data from controller
            double ly = -gamepad1.left_stick_y; // forward/backward driving
            double lx = gamepad1.left_stick_x; // strafing
            double rx = gamepad1.right_stick_x; // turning
            boolean button1state = gamepad1.b; // slow mode

            double heading = -robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double adjustedLx = -ly*Math.sin(heading) + lx*Math.cos(heading);
            double adjustedLy = ly*Math.cos(heading) + lx*Math.sin(heading);


            if (button1state && !button1PrevState) {
                slowMode = !slowMode;
                speedMultiplier = (float) (slowMode ? 0.3 : 1.0);
            }

            // Calculate motor powers
            double frontLeftPower = (adjustedLy + adjustedLx + rx)*speedMultiplier;
            double frontRightPower = (adjustedLy - adjustedLx - rx)*speedMultiplier;
            double backLeftPower = (adjustedLy - adjustedLx + rx)*speedMultiplier;
            double backRightPower = (adjustedLy + adjustedLx - rx)*speedMultiplier;

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

            button1PrevState = button1state;


            /* Telemetry
            telemetry.addData("Status", "RUNNING");
            telemetry.addData("FrontLeft Motor Power", frontLeftPower);
            telemetry.addData("FrontRight Motor Power", frontRightPower);
            telemetry.addData("BackLeft Motor Power", backLeftPower);
            telemetry.addData("BackRight Motor Power", backRightPower);
            telemetry.update();*/
            //Im stupi
        }
    }
}