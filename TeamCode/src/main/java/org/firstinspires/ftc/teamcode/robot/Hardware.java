package org.firstinspires.ftc.teamcode.robot;

// motors
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

// import imu
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

// import ftclib odometry
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

// ftc lib gamepad
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.util.ElapsedTime;


public class Hardware {
    public DcMotor frontLeft, frontRight, backLeft, backRight, intakeMotor;
    public IMU imu;
    public ElapsedTime timer;

    // odometry setup TODO: USE MILLIMETRES AS UNIT
//    public static final double TRACK_WIDTH = 373.38;
//    public static final double CENTER_WHEEL_OFFSET = -2.1; // perpendicular distance from center of rotation on robot IN MILLIMETRES TODO: NEEDS TUNING
//    public static final double WHEEL_DIAMETER = 48; // diameter of odometry wheels IN MILLIMETRES  TODO: NEEDS TUNING
//    public static final double TICKS_PER_REVOLUTION = 8192; // how many ticks the encoder has in a full rotation TODO: NEEDS TUNING
//    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REVOLUTION; // distance per tick IN MILLIMETRES TODO: NEEDS TUNING

//    public MotorEx leftOdom, rightOdom, perpenOdom;
//    public HolonomicOdometry odometry;

    // get gamepad objects

    // Init hardwareMaps
    public void initialize(HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        imu = hardwareMap.get(IMU.class, "imu");

        // Set motor zero power behavior to brake instead of move freely
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set directions for each motor
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Initialize the IMU
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);

//        // setup odometry
//        leftOdom.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
//        rightOdom.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
//        perpenOdom.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
//
//        leftOdom.setInverted(false);
//        rightOdom.setInverted(true);
//        perpenOdom.setInverted(false);
//
//        odometry = new HolonomicOdometry(
//                leftOdom::getDistance,
//                rightOdom::getDistance,
//                perpenOdom::getDistance,
//                TRACK_WIDTH, CENTER_WHEEL_OFFSET
//        );

        timer.reset();
    }

}