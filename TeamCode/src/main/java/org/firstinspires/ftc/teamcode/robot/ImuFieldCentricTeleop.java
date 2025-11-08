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
        double outtakePower = 0;
        boolean storageState = false;
        boolean dpu2_prevState = false;
        boolean dpd2_prevState = false;
        boolean a2_prevState = false;
        double storageStopTime = 0;

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
            boolean a2state = gamepad2.a;
            boolean b2state = gamepad2.b;
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

            // intake
            double intakePower;
            if (ly2 >= 0.5) {intakePower = 1;}
            else if (ly2 <= -0.5) {intakePower = -1;}
            else {intakePower=0;}
            robotHardware.intakeMotor.setPower(intakePower);

//            // outtake power control ry2
//            if (ry2 >= 0.75) {outtakePower = 0.7;} // full power outtake
//            else if (ry2 >= 0.5) {outtakePower = 0.5;} // half power outtake
//            else if (ry2 >= 0.25) {outtakePower = 0.3;} // low power outtake
//            else {outtakePower = 0;} // stop outtake

            // outtake power control dpad
            if (dpu2 && !dpu2_prevState) {
                outtakePower += 0.05; // Increment power percentage only once per press
            } else if (dpd2 && !dpd2_prevState) {
                outtakePower -= 0.05; // Decrement power percentage only once per press
            }

            dpu2_prevState = dpu2;
            dpd2_prevState = dpd2;

            outtakePower = Math.max(0, Math.min(0.8, outtakePower));

            double targetRpm = outtakePower * Hardware.OUTTAKE_MAX_RPM;
            double targetTps = (targetRpm / 60.0) * Hardware.OUTTAKE_TPR;

            robotHardware.outtakeMotor.setVelocity(targetTps);

            // set outtake power with storage control
            // if (lt2state >= 0.5) {outtakePower *= 0.45;} else {outtakePower*=0.7;}
            /*if (home2state && !home2prevState) {
                storageState = !storageState;
                if (storageState) {robotHardware.storage.setPower(1);}
                else {robotHardware.storage.setPower(0);}
            } home2prevState = home2state;*/


            double currentTime = robotHardware.timer.milliseconds();
            if (a2state && !a2_prevState && !storageState) {
                storageState = true;
                robotHardware.storage.setPower(1.0);
                storageStopTime = currentTime + 3000;
            }
            if (storageState && currentTime >= storageStopTime) {
                storageState = false;
                robotHardware.storage.setPower(0.0);
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