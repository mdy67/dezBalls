package org.firstinspires.ftc.teamcode.GEN4.Misc;

// ❗ IMPORTANT: Use the FTC SDK Range, not android.util.Range
import com.qualcomm.robotcore.util.Range;

public class DTPID {

    public double kP;
    public double kD;

    public DTPID(double P, double D) {
        kP = P;
        kD = D;
    }

    double lastError = 0;
    int counter = 0;

    double lastLoopTime = System.nanoTime();
    double loopTime = 0.0;
    double currentTime = System.nanoTime();

    double proportion, derivative;

    double output = 0.0;

    /**
     * Calculates new PD output power.
     *
     * @param error The control error (target - current)
     * @param max Maximum output value
     * @return PD output clipped to min/max
     */
    public double newPDPower(double error, double max) {

        if (counter == 0) {
            lastLoopTime = System.nanoTime() - 10_000_000;  // 10ms
        }

        currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime) / 1_000_000_000.0;
        lastLoopTime = currentTime;

        proportion = kP * error;
        derivative = kD * (error - lastError) / loopTime;

        lastError = error;
        counter++;

        output = Range.clip(proportion + derivative, -max, max);

        // motor power output
        return Math.sqrt(output) * Math.signum(output);
    }
}
