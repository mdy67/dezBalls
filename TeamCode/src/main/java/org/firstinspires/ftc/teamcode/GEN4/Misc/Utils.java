package org.firstinspires.ftc.teamcode.GEN4.Misc;

public class Utils {
    public static double headingClip(double value) {
        while(value >= Math.PI) {
            value -= 2*Math.PI;
        }
        while(value <= -Math.PI) {
            value += 2*Math.PI;
        }
        return value;
    }
}
