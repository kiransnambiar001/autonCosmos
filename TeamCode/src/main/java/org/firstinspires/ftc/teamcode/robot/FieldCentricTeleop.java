package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // For linear OpModes
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; // For TeleOp OpModes

import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="FieldCentric TeleOp", group="LinearOpMode")

public class FieldCentricTeleop  extends LinearOpMode {


    // Create hardware object
    Hardware robotHardware = new Hardware();

    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        // use hardware class to initialize everything (code in Hardware file)
        robotHardware.initialize(hardwareMap);

        // update telemetry to show INITIALIZED status
        telemetry.addData("Status", "INITIALIZED");
        telemetry.update();

        timer.reset();
        // wait for user to press start button
        waitForStart();

        float speedMultiplier = 1.0f;
        boolean button1prevState = false;
        boolean slowMode = false;

        // start OpMode loop
        while (opModeIsActive()) {
            robotHardware.odometry.updatePose();

            Pose2d currentPose = robotHardware.odometry.getPose();
            double odomHeading = currentPose.getHeading();

            // get data from controller
            double ly = -gamepad1.left_stick_y; // forward/backward driving
            double lx = gamepad1.left_stick_x; // strafing
            double rx = gamepad1.right_stick_x; // turning
            boolean button1state = gamepad1.b; // slow mode

            double imuHeading = -robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double adjustedLx = -ly*Math.sin(imuHeading) + lx*Math.cos(imuHeading);
            double adjustedLy = ly*Math.cos(imuHeading) + lx*Math.sin(imuHeading);

            if (button1state && !button1prevState) {
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

            /* Telemetry
            telemetry.addData("Status", "RUNNING");
            telemetry.addData("FrontLeft Motor Power", frontLeftPower);
            telemetry.addData("FrontRight Motor Power", frontRightPower);
            telemetry.addData("BackLeft Motor Power", backLeftPower);
            telemetry.addData("BackRight Motor Power", backRightPower);
            telemetry.update();*/

            button1prevState = button1state;
        }
    }
}