package org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotorEx intake;
    private DcMotorEx indexer;

    private double intakeSpeed = 1.0;
    private double indexerSpeed = -1.0;
    private double indexerIdle = -0.4;
    private ColorSensors colorSensors;

    public Intake(HardwareMap hardwareMap, ColorSensors colorSensors) {
        this.intake = hardwareMap.get(DcMotorEx.class, "intake");
        this.indexer = hardwareMap.get(DcMotorEx.class, "indexer");

        this.colorSensors = colorSensors;

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        indexer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        indexer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Set speeds (optional) */
    public void setSpeeds(double intakeSpeed, double indexerSpeed) {
        this.intakeSpeed = intakeSpeed;
        this.indexerSpeed = indexerSpeed;
    }

    /** Call this each loop to control intake/indexer */
    public void teleUpdate(boolean intakeIn, boolean intakeOut) {
        double numBalls = colorSensors.numBalls(); // current ball count
        double currentTime = System.nanoTime() / 1e9; // seconds

        if (intakeIn) {
            intake.setPower(intakeSpeed);
            indexer.setPower(indexerSpeed);
        } else if (intakeOut) {
            intake.setPower(-intakeSpeed);
            indexer.setPower(-indexerSpeed);
        } else if (numBalls > 0) {
            // Auto-run indexer if balls are detected
            intake.setPower(0);
            indexer.setPower(indexerSpeed);
        } else {
            // Check delay: keep indexer running for INDEXER_DELAY seconds after last ball
            intake.setPower(0);
            indexer.setPower(indexerIdle); // stop
        }
    }

    /** Stop all motors immediately */
    public void stopIntakeSystem() { intake.setPower(0); indexer.setPower(0);}
    /** Run indexer manually at a given power (for shooting) */
    public void runIndexerOnly(double power) {
        indexer.setPower(power);
    }
    public void runIntakeOnly(double power) { intake.setPower(power); }
    /** Getter for indexer speed */
    public double getIndexerSpeed() {
        return indexerSpeed;
    }
}
