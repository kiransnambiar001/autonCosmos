package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // For linear OpModes
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; // For TeleOp OpModes
import com.qualcomm.robotcore.hardware.DcMotor; // For DC motors
import com.qualcomm.robotcore.hardware.HardwareMap;

//import IMU
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

// create classes for motors

@TeleOp(name="ImuFieldCentric TeleOp", group="LinearOpMode")

public class ImuFieldCentricTeleop  extends LinearOpMode {


    // Create hardware object
    Hardware robotHardware = new Hardware();


    // Init hardwareMaps for each motor


    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware.initialize(hardwareMap);

        // update telemetry to show INITIALIZED status
        telemetry.addData("Status", "INITIALIZED");
        telemetry.update();

        // wait for user to press start button
        waitForStart();

        boolean home1prevState = false;
        boolean options1prevState = false;
        boolean fieldCentric = true;
        robotHardware.imu.resetYaw();

        // start OpMode loop
        while (opModeIsActive()) {
            // gamepad1
            double ly1 = -gamepad1.left_stick_y; // forward/backward driving
            double lx1 = gamepad1.left_stick_x; // strafing
            double rx1 = gamepad1.right_stick_x; // turning
            boolean home1state = gamepad1.guide; // to reset yaw value on gyro
            boolean options1state = gamepad1.options; // field centric
            double lt1state = gamepad1.left_trigger; // toggle slowmode


            // gamepad2
            double ly2 = gamepad1.left_stick_y;
            double ry2 = gamepad1.left_stick_x;
            double lt2state = gamepad2.left_trigger; // slow mode


            double imuHeading = robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            // field centric toggles
            if (home1state && !home1prevState && fieldCentric) {
                robotHardware.imu.resetYaw();
            } home1prevState = home1state;
            if (options1state && !options1prevState) {
                fieldCentric = !fieldCentric;
            } options1prevState = options1state;
            updateDriveBase(ly1, lx1, rx1, lt1state, imuHeading, fieldCentric);

            // intake
            double intakePower;
            if (ly2 >= 0.5) {intakePower = 1;}
            else if (ly2 <= -0.5) {intakePower = -1;}
            else {intakePower=0;}
            robotHardware.intakeMotor.setPower(intakePower);


            // outtake
            double outtakePower = ry2;
            if (lt2state >= 0.5) {outtakePower *= 0.3;} else {outtakePower*=0.7;}
            robotHardware.outtakeMotor.setPower(outtakePower);
        }
    }
    private void updateDriveBase(double ly, double lx, double rx, double lt1state, double imuHeading, boolean fieldCentric) {
        double speedMultiplier;
        if (lt1state > 0.5) {speedMultiplier = 0.3;}
        else {speedMultiplier = 1.0;}


        double adjLy, adjLx;

        if (fieldCentric) {
            adjLx = -ly * Math.sin(imuHeading) + lx * Math.cos(imuHeading);
            adjLy = ly * Math.cos(imuHeading) + lx * Math.sin(imuHeading);
        }
        else {
            adjLx = lx; adjLy = ly;
        }

        double frontLeftPower = (adjLy + adjLx + rx)*speedMultiplier;
        double frontRightPower = (adjLy - adjLx - rx)*speedMultiplier;
        double backLeftPower = (adjLy - adjLx + rx)*speedMultiplier;
        double backRightPower = (adjLy + adjLx - rx)*speedMultiplier;

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

        robotHardware.frontLeft.setPower(frontLeftPower);
        robotHardware.frontRight.setPower(frontRightPower);
        robotHardware.backLeft.setPower(backLeftPower);
        robotHardware.backRight.setPower(backRightPower);
    }
}