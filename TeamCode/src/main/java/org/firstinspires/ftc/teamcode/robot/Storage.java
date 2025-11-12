package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Storage {

    private final Hardware robotHardware;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean isTimedRunActive = false;
    private double stopTime = 0;

    public Storage(Hardware hardware) {
        this.robotHardware = hardware;
    }

    public void runForTime(double power, double durationMs) {
        if (!isTimedRunActive) {
            this.isTimedRunActive = true;
            this.timer.reset();
            this.stopTime = timer.milliseconds() + durationMs;
            this.robotHardware.storage.setPower(power);
        }
    }


    public void run(double power) {
        this.isTimedRunActive = false;
        this.robotHardware.storage.setPower(power);
    }

    public void update() {
        if (isTimedRunActive && timer.milliseconds() >= stopTime) {
            run(0);
        }
    }
}
