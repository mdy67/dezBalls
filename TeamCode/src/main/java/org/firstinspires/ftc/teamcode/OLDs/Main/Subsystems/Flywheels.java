package org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayDeque;
import java.util.Deque;

public class Flywheels {

    private final DcMotorEx shooterL, shooterR;
    private final VoltageSensor battery;

    // Target velocity in rad/s
    private double targetVelocity = 0.0;

    // Proportional gain
    private double kP = 0.0018;

    // Feedforward parameters
    private double kV = 0.00163;
    private double kS = 0.09765;
    private double tunedVoltage = 13.0283;

    // Velocity smoothing buffer
    private final Deque<Double> velBuffer = new ArrayDeque<>();
    private final int BUFFER_SIZE = 18;
    private double smoothedVelocity = 0.0;

    // Last tick/time for velocity calculation
    private double lastTicks = 0.0;
    private double lastTime = 0.0;

    public Flywheels(HardwareMap hardwareMap, VoltageSensor battery) {
        this.battery = battery;

        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastTicks = shooterL.getCurrentPosition();
        lastTime = System.nanoTime() / 1e9;
    }

    /** Set feedforward values and tuned voltage. */
    public void setFeedforward(double kV, double kS, double tunedVoltage) {
        this.kV = kV;
        this.kS = kS;
        this.tunedVoltage = tunedVoltage;
    }

    /** Set proportional gain. */
    public void setKP(double kP) {
        this.kP = kP;
    }

    /** Set target velocity in rad/s. */
    public void setTargetVelocity(double radPerSec) {
        this.targetVelocity = radPerSec;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** Compute smoothed velocity in rad/s using shooterL encoder. Call frequently. */
    public void updateVelocity() {
        double currentTicks = shooterL.getCurrentPosition();
        double currentTime = System.nanoTime() / 1e9;

        double deltaTicks = currentTicks - lastTicks;
        double deltaTime = currentTime - lastTime;

        // Convert ticks/sec to rad/s (bare motor, 28 ticks/rev)
        double instVel = deltaTime > 0 ? (deltaTicks / deltaTime) * 2 * Math.PI / 28.0 : 0.0;

        velBuffer.addLast(instVel);
        if (velBuffer.size() > BUFFER_SIZE) velBuffer.removeFirst();

        smoothedVelocity = velBuffer.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

        lastTicks = currentTicks;
        lastTime = currentTime;
    }

    /** Get smoothed velocity in rad/s. */
    public double getVelocityRadPerSec() {
        return smoothedVelocity;
    }

    /** Get smoothed velocity in RPM. */
    public double getVelocityRPM() {
        return smoothedVelocity * 60.0 / (2.0 * Math.PI);
    }

    /** Get last applied motor power. */
    public double getPower() {
        return shooterL.getPower();
    }

    /** Update motor power using feedforward + kP control. Call frequently. */
    public void update() {
        updateVelocity();

        double voltage = (battery != null) ? battery.getVoltage() : 13.0; // fallback
        double feedforwardPower = (kV * targetVelocity + kS) * tunedVoltage / voltage;

        // Proportional correction
        double error = targetVelocity - smoothedVelocity;
        double power = feedforwardPower + (kP * error);

        power = Range.clip(power, 0.0, 1.0);

        shooterL.setPower(power);
        shooterR.setPower(power);
    }

    /** Immediately stop flywheel. */
    public void stop() {
        shooterL.setPower(0.0);
        shooterR.setPower(0.0);
    }
}
