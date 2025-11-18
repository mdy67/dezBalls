package org.firstinspires.ftc.teamcode.GEN4.Misc;

import com.qualcomm.robotcore.util.Range;

public class DTPID {

    public double kP;
    public double kD;

    public DTPID(double P, double D) {
        kP = P;
        kD = D;
    }
    double integral = 0;
    double lastError = 0;
    int counter = 0;

    double lastLoopTime = System.nanoTime();
    double loopTime = 0.0;
    double currentTime = System.nanoTime();
    double proportion, derivative;

    public double newPDPower(double error, double min, double max) {
        if (counter == 0) {
            lastLoopTime = System.nanoTime() - 10000000;
        }

        currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime) / 10000000;

        proportion = kP * error;
        derivative = kD * (error - lastError) / loopTime;

        lastError = error;
        counter ++;

        return Range.clip(proportion + derivative, min, max);
    }
}
