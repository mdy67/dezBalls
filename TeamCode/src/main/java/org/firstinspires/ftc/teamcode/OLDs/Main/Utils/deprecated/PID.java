package org.firstinspires.ftc.teamcode.OLDs.Main.Utils.deprecated;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double kP, kI, kD;
    private double lastError = 0;
    private double integral = 0;
    private boolean enabled = false;
    private ElapsedTime timer = new ElapsedTime();

    // Output limits, can be positive/negative for motors
    private double minOutput = -1;
    private double maxOutput = 1;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer.reset();
    }

    public double update(double currentValue, double targetValue) {
        if (!enabled) {
            reset();
            return 0;
        }

        double error = targetValue - currentValue;
        double deltaTime = timer.seconds();
        timer.reset();

        if (deltaTime <= 0) return 0;

        integral += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp to output limits (allows negative values)
        output = Math.max(minOutput, Math.min(maxOutput, output));
        return output;
    }

    public void setEnabled(boolean enable) {
        if (enable && !this.enabled) {
            timer.reset();
            lastError = 0;
            integral = 0;
        }
        this.enabled = enable;
    }

    public boolean isEnabled() { return enabled; }

    public void reset() {
        lastError = 0;
        integral = 0;
        timer.reset();
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }
}
