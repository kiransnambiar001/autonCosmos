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

        float speedMultiplier = 1.0f;
        boolean home1prevState = false;
        boolean options1prevState = false;
        boolean slowMode = false;
        boolean fieldCentric = true;
        robotHardware.imu.resetYaw();

        // start OpMode loop
        while (opModeIsActive()) {
            // get data from controller
            double ly = -gamepad1.left_stick_y; // forward/backward driving
            double lx = gamepad1.left_stick_x; // strafing
            double rx = gamepad1.right_stick_x; // turning
            boolean home1state = gamepad1.guide; // to reset yaw value on gyro
            boolean options1state = gamepad1.options; // field centric
            double rt1state = gamepad1.right_trigger; // to power intake variably TODO: set to second gamepad later if needed
            double imuHeading = robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double lt1state = gamepad1.left_trigger; // toggle slowmode
            boolean rb1state = gamepad1.right_bumper; // reverse intake
            double adjLy, adjLx;

            if (lt1state > 0.5) {slowMode = true;}
            else {slowMode = false;}

            speedMultiplier = (float) (slowMode ? 0.3 : 1.0);

            if (home1state && !home1prevState && fieldCentric) {
                robotHardware.imu.resetYaw();
            } home1prevState = home1state;

            if (options1state && !options1prevState) {
                fieldCentric = !fieldCentric;
            } options1prevState = options1state;


            if (fieldCentric) {
                adjLx = -ly * Math.sin(imuHeading) + lx * Math.cos(imuHeading);
                adjLy = ly * Math.cos(imuHeading) + lx * Math.sin(imuHeading);
            }
            else {
                adjLx = lx; adjLy = ly;
            }



            // Calculate motor powers
            double intakePower;
            if (rt1state >= 0.5) {intakePower = 1;}

            else {intakePower=0;}
            intakePower = (float) (rb1state ? -(intakePower): intakePower);

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

            // set motor power based on values
            robotHardware.intakeMotor.setPower(intakePower);
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
            //Im stupi
        }
    }
}