package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
        boolean home2prevState = false;
        double intakePower;

        // ...
        double outtakePower = 0;
        double closeShotPower = 0.45;
        double farShotPower = 0.70;
        boolean storageState = false;

        boolean dpu2_prevState = false;
        boolean dpd2_prevState = false;
        boolean a2_prevState = false;
        boolean rt2_prevState = false;
        boolean outtakeState = false;
        double storageStopTime = 0;
        double outtakeStopTime = 0;

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
            double ly2 = gamepad2.left_stick_y;
            double ry2 = gamepad2.right_stick_y;
            double lt2state = gamepad2.left_trigger; // slow mode
            double rt2state= gamepad2.right_trigger;
            boolean a2state = gamepad2.a; // storage on/off
            boolean b2state = gamepad2.b; // outtake preset for close shoot
            boolean x2state = gamepad2.x; // outtake preset for far shoot
            boolean home2state = gamepad2.options;
            boolean dpu2 = gamepad2.dpad_up;
            boolean dpd2 = gamepad2.dpad_down;



            double imuHeading = robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            // field centric toggles
            if (home1state && !home1prevState && fieldCentric) {
                robotHardware.imu.resetYaw();
            } home1prevState = home1state;
            if (options1state && !options1prevState) {
                fieldCentric = !fieldCentric;
            } options1prevState = options1state;
            updateDriveBase(ly1, lx1, rx1, lt1state, imuHeading, fieldCentric);

            // intake *************************
            if (ly2 >= 0.5) {intakePower = 1;}
            else if (ly2 <= -0.5) {intakePower = -1;}
            else {intakePower=0;}
            robotHardware.intakeMotor.setPower(intakePower);
            // intake *************************


            // outtake *********************************************************************************
            final double OUTTAKE_IDLE_POWER = 0.2;
            final double OUTTAKE_MAX_POWER = 0.7;
            // outtake power control dpad
            if (dpu2 && !dpu2_prevState && outtakePower <= OUTTAKE_MAX_POWER) {
                outtakePower += 0.05;
            } else if (dpd2 && !dpd2_prevState && outtakePower >= OUTTAKE_IDLE_POWER) {
                outtakePower -= 0.05;
            }

            dpu2_prevState = dpu2;
            dpd2_prevState = dpd2;

            double targetRpm = outtakePower * Hardware.OUTTAKE_MAX_RPM;
            double targetTps = (targetRpm / 60) * Hardware.OUTTAKE_TPR;

            if (b2state)
            {
                outtakePower = 0.45;
                robotHardware.outtakeMotor.setVelocity(targetTps);
            }
            else if (x2state)
            {
                outtakePower = 0.7;
                robotHardware.outtakeMotor.setVelocity(targetTps);
            }
            else
            {
                outtakePower = 0.2;
            }

            // outtake *********************************************************************************

            /*double currentOuttakeTime = robotHardware.timer.milliseconds();
            if (rt2state > 0.5 && !rt2_prevState && !outtakeState) {
                outtakeState = true;
                robotHardware.outtakeMotor.setVelocity(targetTps);
                outtakeStopTime = currentOuttakeTime + 2000;
            }
            if (outtakeState && currentOuttakeTime >= outtakeStopTime) {
                outtakeState = false;
                robotHardware.outtakeMotor.setPower(OUTTAKE_CONST_POWER);
            }
            rt2_prevState = rt2state > 0.5;*/

            double currentStorageTime = robotHardware.timer.milliseconds();
            if (a2state && !a2_prevState && !storageState) {
                storageState = true;
                robotHardware.storage.setPower(1);
                storageStopTime = currentStorageTime + 1500;
            }
            if (storageState && currentStorageTime >= storageStopTime) {
                storageState = false;
                robotHardware.storage.setPower(0);
            }
            a2_prevState = a2state;

            telemetry.addData("Outtake Motor",outtakePower);
            telemetry.addData("Target Velocity (tps)", targetTps);
            telemetry.addData("Actual Velocity (tps)", robotHardware.outtakeMotor.getVelocity());
            telemetry.addData("Storage Motor", storageState);
            telemetry.addData("Field Centric On?", fieldCentric ? "on":"off");
            telemetry.update();

        }
    }
    private void updateDriveBase(double ly, double lx, double rx, double lt1state, double imuHeading, boolean fieldCentric) {
        double speedMultiplier;
        if (lt1state > 0.5) {speedMultiplier = 0.3;}
        else {speedMultiplier = 1.0;}


        double adjLy, adjLx;

        if (fieldCentric) {
            adjLx = lx * Math.cos(-imuHeading) - ly * Math.sin(-imuHeading); // THIS LINE WAS MISSING
            adjLy = lx * Math.sin(-imuHeading) + ly * Math.cos(-imuHeading);
        }
        else {
            adjLx = lx;
            adjLy = ly;
        }

        double frontLeftPower = (adjLy + adjLx + rx)*speedMultiplier;
        double frontRightPower = (adjLy - adjLx - rx)*speedMultiplier;
        double backLeftPower = (adjLy - adjLx + rx)*speedMultiplier;
        double backRightPower = (adjLy + adjLx - rx)*speedMultiplier;

        // Get the motor with the highest power
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