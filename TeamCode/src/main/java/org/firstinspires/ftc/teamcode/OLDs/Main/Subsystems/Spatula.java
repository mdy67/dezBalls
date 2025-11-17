package org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Spatula {
    private final Servo spatula;
    private static final double ENGAGED = 0.55;   // ON position
    private static final double DISENGAGED = 0.604; // OFF position

    public Spatula(HardwareMap hardwareMap) {
        spatula = hardwareMap.get(Servo.class, "spatula");
    }

    public void spatulaON() {
        spatula.setPosition(ENGAGED);
    }

    public void spatulaOFF() {
        spatula.setPosition(DISENGAGED);
    }

    public double getPosition() {
        return spatula.getPosition();
    }
}
