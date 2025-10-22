package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;



@Autonomous(name="RR Auto", group="Autonomous")
public class RobotAuton {

    public Hardware robotHardware = new Hardware();

    public class Intake {
        public class IntakeIn implements Action {
            private boolean initialized = false;
            private final double intakeTimer = 2000;
            private double intakePrevTime = 0;
            private double currentTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                currentTime = robotHardware.timer.milliseconds();
                if (!initialized) {
                    robotHardware.intakeMotor.setPower(0.8);
                    initialized = true;
                    intakePrevTime = currentTime;
                }


                if (currentTime-intakePrevTime >= intakeTimer) {
                    return true;
                } else {
                    robotHardware.intakeMotor.setPower(0);
                    return false;
                }
            }
        }
        public Action intakeIn() {
            return new IntakeIn();
        }

        public class IntakeOut implements Action {
            private boolean initialized = false;
            private final double intakeTimer = 2000;
            private double intakePrevTime = 0;
            private double currentTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                currentTime = robotHardware.timer.milliseconds();
                if (!initialized) {
                    robotHardware.intakeMotor.setPower(-0.8);
                    initialized = true;
                    intakePrevTime = currentTime;
                }


                if (currentTime-intakePrevTime >= intakeTimer) {
                    return true;
                } else {
                    robotHardware.intakeMotor.setPower(0);
                    return false;
                }
            }
        }
        public Action intakeOut() {
            return new IntakeOut();
        }
    }
    public void runOpMode() throws InterruptedException {

    }
}
