package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Colors {

    private ColorSensor color1, color2, color3;
    private Servo led1, led2, led3;

    // Thresholds
    private static final double PRESENCE = 80;   // min red to detect a ball
    private static final double PURPLE_T = 200;  // min red to classify as purple

    public enum BallState {
        EMPTY,
        GREEN,
        PURPLE
    }

    private BallState state1 = BallState.EMPTY;
    private BallState state2 = BallState.EMPTY;
    private BallState state3 = BallState.EMPTY;

    public Colors(HardwareMap hardwareMap) {
        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        color3 = hardwareMap.get(ColorSensor.class, "color3");

        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");
        led3 = hardwareMap.get(Servo.class, "led3");
    }

    /** Converts one sensor reading into EMPTY / GREEN / PURPLE */
    private BallState classify(ColorSensor sensor) {
        double r = sensor.red();

        if (r < PRESENCE) {
            return BallState.EMPTY;
        }
        if (r < PURPLE_T) {
            return BallState.GREEN;
        }
        return BallState.PURPLE;
    }

    /** Sets the LED servo based on the ball state */
    private void applyLED(Servo led, BallState state) {
        switch (state) {
            case EMPTY:
                led.setPosition(0.0); // LED OFF
                break;
            case GREEN:
                led.setPosition(0.47); // LED GREEN
                break;
            case PURPLE:
                led.setPosition(0.722); // LED PURPLE
                break;
        }
    }

    /** Call this every loop */
    public void update() {
        state1 = classify(color1);
        state2 = classify(color2);
        state3 = classify(color3);

        applyLED(led1, state1);
        applyLED(led2, state2);
        applyLED(led3, state3);
    }

    // ===========================
    // ACCESSORS / HELPERS
    // ===========================

    public BallState getBall1() { return state1; }
    public BallState getBall2() { return state2; }
    public BallState getBall3() { return state3; }

    public boolean hasBall1() { return state1 != BallState.EMPTY; }
    public boolean hasBall2() { return state2 != BallState.EMPTY; }
    public boolean hasBall3() { return state3 != BallState.EMPTY; }
    public boolean hasBall() { return state1 != BallState.EMPTY || state2 != BallState.EMPTY || state3 != BallState.EMPTY; }


    public boolean isPurple1() { return state1 == BallState.PURPLE; }
    public boolean isPurple2() { return state2 == BallState.PURPLE; }
    public boolean isPurple3() { return state3 == BallState.PURPLE; }

    public boolean isGreen1() { return state1 == BallState.GREEN; }
    public boolean isGreen2() { return state2 == BallState.GREEN; }
    public boolean isGreen3() { return state3 == BallState.GREEN; }
}
