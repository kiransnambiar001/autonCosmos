package org.firstinspires.ftc.teamcode.robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous(name="RobotAutonClose BLUE", group="Robot")

public class RobotAutonCloseBlue extends LinearOpMode {

    Hardware robotHardware = new Hardware();

    @Override
    public void runOpMode() {

        robotHardware.initialize(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            drive(0.5,0.5,0.5,0.5, 1000); // forward until CLOSE shoot zone
            drive(-0.5,0.5,-0.5,0.5,500); // turn 45 degrees to the right

            robotHardware.intakeMotor.setPower(0.7);
            robotHardware.outtakeMotor.setPower(0.7);
            robotHardware.intakeMotor.setPower(0);
            robotHardware.outtakeMotor.setPower(0);


        }
    }

    private void drive(double flp, double frp, double blp, double brp, long millis) {
        robotHardware.frontLeft.setPower(flp);
        robotHardware.frontRight.setPower(frp);
        robotHardware.backLeft.setPower(blp);
        robotHardware.backLeft.setPower(brp);
        sleep(millis);
        robotHardware.frontLeft.setPower(0);
        robotHardware.frontRight.setPower(0);
        robotHardware.backLeft.setPower(0);
        robotHardware.backLeft.setPower(0);
    }
}