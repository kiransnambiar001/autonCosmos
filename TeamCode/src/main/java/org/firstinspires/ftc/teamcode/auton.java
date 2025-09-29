package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.imu.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "TestAutonomousOpMode", group = "LinearOpMode")
public class main extends LinearOpMode {

    // IMU and motor declarations
    public IMU imu;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    // PID-style constants
    public final double HEADING_KP = 0.01; // Rotation tuning
    public final double TOLERANCE = 1.0; // Position tolerance (not used in this example)
    public final double ANGLE_TOLERANCE = 2.0; // Degrees

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        imu = hardwareMap.get(IMU.class, "imu");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // IMU orientation setup
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Drive forward (Y=1), no strafe (X=0), no rotation, at 50% speed
            drive(0, 1, 0, 0.5f);

            // Pause
            sleep(1000);

            // Strafe right (X=1), no forward (Y=0), no rotation
            drive(1, 0, 0, 0.5f);

            // Pause
            sleep(1000);

            // Rotate clockwise in place
            drive(0, 0, 1, 0.5f);

            sleep(1000);

            // Stop all movement
            drive(0, 0, 0, 0f);
        }
    }

    /**
     * Drives the robot based on x (strafe), y (forward/back), and rotation input.
     * Inputs are assumed to be in range [-1, 1].
     */
    public void drive(double x, double y, double rotation, float speed) {
        // Get current yaw angle from IMU
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw(AngleUnit.RADIANS);

        // Rotate input for field-centric control
        double rotX = x * Math.cos(-yaw) - y * Math.sin(-yaw);
        double rotY = x * Math.sin(-yaw) + y * Math.cos(-yaw);

        // Calculate raw motor powers
        double frontLeftPower = rotY + rotX + rotation;
        double frontRightPower = rotY - rotX - rotation;
        double backLeftPower = rotY - rotX + rotation;
        double backRightPower = rotY + rotX - rotation;

        // Normalize motor powers
        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Scale with speed
        frontLeft.setPower(frontLeftPower * speed);
        frontRight.setPower(frontRightPower * speed);
        backLeft.setPower(backLeftPower * speed);
        backRight.setPower(backRightPower * speed);
    }
}


