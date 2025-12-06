package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Wait {
    private ElapsedTime timer = new ElapsedTime();
    private double duration = 0;
    private boolean active = false;

    // Start waiting for a given duration in seconds
    public void waitSeconds(double seconds) {
        duration = seconds;
        timer.reset();
        active = true;
    }

    // Update method to be called each loop (required for consistency)
    public void update() {
        // Nothing needed here for ElapsedTime, but placeholder if needed
    }

    // Returns true if the wait is complete
    public boolean isFinished() {
        if (!active) return false;
        if (timer.seconds() >= duration) {
            active = false;
            return true;
        }
        return false;
    }

    // Returns true if wait is currently active
    public boolean isActive() {
        return active;
    }

    // Reset timer and deactivate
    public void reset() {
        active = false;
        timer.reset();
    }
}
