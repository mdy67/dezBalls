package org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Arms {

    // Servos
    private Servo arm1, arm2, arm3;

    // Positions
    public static double ARM_1_UP = 0.55;
    public static double ARM_1_DOWN = 0.2;
    public static double ARM_2_UP = 0.67;
    public static double ARM_2_DOWN = 0.34;
    public static double ARM_3_UP = 0.67;
    public static double ARM_3_DOWN = 0.34;
    public static double ARM_3_RAPID = 0.54;
    private boolean arm3rapid = false;

    // Flick duration (seconds)
    public static double FLICK_HOLD_TIME = 0.35;

    // Flick state & timers
    private boolean flickArm1 = false;
    private boolean flickArm2 = false;
    private boolean flickArm3 = false;

    private double flickArm1Start = 0;
    private double flickArm2Start = 0;
    private double flickArm3Start = 0;

    public Arms(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");

        // Initialize all arms to down
        arm1.setPosition(ARM_1_DOWN);
        arm2.setPosition(ARM_2_DOWN);
        arm3.setPosition(ARM_3_DOWN);
    }

    /** Call every loop to teleUpdate arms */
    public void update() {
        double currentTime = System.nanoTime() / 1e9;

        // Auto-reset after flick hold time
        if (flickArm1 && currentTime - flickArm1Start > FLICK_HOLD_TIME) flickArm1 = false;
        if (flickArm2 && currentTime - flickArm2Start > FLICK_HOLD_TIME) flickArm2 = false;
        if (flickArm3 && currentTime - flickArm3Start > FLICK_HOLD_TIME) flickArm3 = false;

        // Update servo positions
        arm1.setPosition(flickArm1 ? ARM_1_UP : ARM_1_DOWN);
        arm2.setPosition(flickArm2 ? ARM_2_UP : ARM_2_DOWN);
        arm3.setPosition(flickArm3 ? ARM_3_UP : (arm3rapid ? ARM_3_RAPID : ARM_3_DOWN));
    }

    /** Trigger a flick for each arm */
    public void flickArm1() {
        flickArm1 = true;
        flickArm1Start = System.nanoTime() / 1e9;
    }

    public void flickArm2() {
        flickArm2 = true;
        flickArm2Start = System.nanoTime() / 1e9;
    }

    public void flickArm3() {
        flickArm3 = true;
        flickArm3Start = System.nanoTime() / 1e9;
    }

    public void arm3rapid(boolean gurt) {
        arm3rapid = gurt;
    }

    /** Reset all arms immediately */
    public void reset() {
        flickArm1 = false;
        flickArm2 = false;
        flickArm3 = false;
        update();
    }


    /** Telemetry helpers */
    public double getArm1Pos() { return arm1.getPosition(); }
    public double getArm2Pos() { return arm2.getPosition(); }
    public double getArm3Pos() { return arm3.getPosition(); }
}
