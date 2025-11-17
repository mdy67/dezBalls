package org.firstinspires.ftc.teamcode.GEN4.Misc;

public class Utils {
    public static double headingClip(double value) {
        while(value >= 180) {
            value -= 360;
        }
        while(value <= -180) {
            value += 360;
        }
        return value;
    }
}
