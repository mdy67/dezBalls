package org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems;
import com.qualcomm.robotcore.hardware.ColorSensor; // V3
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensors {
    public static double numBalls;
    public static final double PRESENCE_THRESHOLD = 150;
    ColorSensor colorSensor1, colorSensor2, colorSensor3;

    public ColorSensors (HardwareMap hardwareMap) {
        colorSensor1 = hardwareMap.get(ColorSensor.class, "color1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "color2");
        colorSensor3 = hardwareMap.get(ColorSensor.class, "color3");

    }

    private void getNumBalls() {
        numBalls = 0;
        if (colorSensor1.red() > PRESENCE_THRESHOLD) numBalls++;
        if (colorSensor2.red() > PRESENCE_THRESHOLD) numBalls++;
        if (colorSensor3.red() > PRESENCE_THRESHOLD) numBalls++;
    }

    public double numBalls() {
        getNumBalls();
        return numBalls;
    }

    public void update() {
        getNumBalls();
    }
}
