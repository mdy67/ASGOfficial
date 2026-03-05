package org.firstinspires.ftc.teamcode.GEN4.OLDsubsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Wait {

    private ElapsedTime timer = new ElapsedTime();
    private double duration = 0;
    private boolean active = false;

    /** Start a wait for a given number of seconds */
    public void waitSeconds(double seconds) {
        duration = seconds;
        timer.reset();
        active = true;
    }

    /** Call this every loop to update wait state */
    public void update() {
        if (active && timer.seconds() >= duration) {
            active = false;
        }
    }

    /** Returns true if a wait is currently active */
    public boolean isActive() {
        return active;
    }

    /** Returns true if the wait has finished */
    public boolean isFinished() {
        return !active && duration > 0;
    }
}
