package org.firstinspires.ftc.teamcode.GEN4.Misc;

// ❗ IMPORTANT: Use the FTC SDK Range, not android.util.Range
import com.qualcomm.robotcore.util.Range;

public class DTPID {

    public double kP;
    public double kD;

    // Constructor for PD gains
    public DTPID(double P, double D) {
        kP = P;
        kD = D;
    }

    // Internal variables
    double lastError = 0;
    int counter = 0;

    // Timing variables
    double lastLoopTime = System.nanoTime();
    double loopTime = 0.0;
    double currentTime = System.nanoTime();

    // Output components
    double proportion, derivative;

    /**
     * Calculates new PD output power.
     *
     * @param error The control error (target - current)
     * @param min Minimum output value
     * @param max Maximum output value
     * @return PD output clipped to min/max
     */
    public double newPDPower(double error, double min, double max) {

        // On first loop, fake a slightly earlier lastLoopTime to avoid divide by zero
        if (counter == 0) {
            lastLoopTime = System.nanoTime() - 10_000_000;  // 10ms
        }

        // Get current time and compute Δt (in seconds)
        currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime) / 1_000_000_000.0; // convert ns → seconds

        // Save time for the next cycle
        lastLoopTime = currentTime;

        // Proportional term
        proportion = kP * error;

        // Derivative term (change in error over time)
        derivative = kD * (error - lastError) / loopTime;

        // Store current error for next derivative calculation
        lastError = error;
        counter++;

        // Return PD output, clipped to provided min/max bounds
        return Range.clip(proportion + derivative, min, max);
    }
}
