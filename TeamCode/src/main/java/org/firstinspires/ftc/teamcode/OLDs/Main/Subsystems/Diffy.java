package org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OLDs.CONFIG.RobotConfig;

@Config
public class Diffy {

    public static double BLUE_GOAL_X = -65;
    public static double BLUE_GOAL_Y = 65;
    public static double RED_GOAL_X = 72;
    public static double RED_GOAL_Y = 72;

    public static double targetL = 0;
    public static double targetR = 0;

    public static double kP_L = 0.00028;
    public static double kP_R = 0.00028;

    public static double minPower = 0;
    public static double maxPower = 1.0;
    public static double toleranceTicks = 175;

    public static double slot1Pos = -5300;
    public static double slot2Pos = -2300;
    public static double slot3Pos = 0;

    public static double angleOffset = 0;
    public static double angleScale = 34.4;

    public static double minAngle = 0;
    public static double maxAngle = 180;

    public CRServo diffyL, diffyR;
    public DcMotorEx encL, encR;

    private double currentSlotBase = 0;

    public Diffy(HardwareMap hardwareMap) {
        diffyL = hardwareMap.get(CRServo.class, "diffyL");
        diffyR = hardwareMap.get(CRServo.class, "diffyR");

        encL = hardwareMap.get(DcMotorEx.class, "intake");
        encR = hardwareMap.get(DcMotorEx.class, "leftBack");
        encL.setDirection(DcMotorSimple.Direction.FORWARD);
        encR.setDirection(DcMotorSimple.Direction.REVERSE);

        diffyL.setDirection(CRServo.Direction.FORWARD);
        diffyR.setDirection(CRServo.Direction.FORWARD);

        resetEncoders();
    }

    public void update() {
        double errorL = targetL - encL.getCurrentPosition();
        double errorR = targetR + encR.getCurrentPosition();

        // Safety: stop once within tolerance
        if (Math.abs(errorL) < toleranceTicks && Math.abs(errorR) < toleranceTicks) {
            diffyL.setPower(0);
            diffyR.setPower(0);
            return;
        }

        // Angle limits
        if (angleOffset < minAngle && errorL > 0 && errorR < 0) {
            errorL = 0;
            errorR = 0;
        }
        if (angleOffset > maxAngle && errorL < 0 && errorR > 0) {
            errorL = 0;
            errorR = 0;
        }

        // PID (proportional only)
        double powerL = Range.clip(errorL * kP_L, -maxPower, maxPower);
        double powerR = Range.clip(errorR * kP_R, -maxPower, maxPower);

        // Deadband to prevent jitter
        if (Math.abs(powerL) < minPower) powerL = 0;
        if (Math.abs(powerR) < minPower) powerR = 0;

        // Apply (mirror right side)
        diffyL.setPower(powerL);
        diffyR.setPower(-powerR);
    }

    public double[] getErrors() {
        return new double[]{
                targetL - encL.getCurrentPosition(),
                targetR - encR.getCurrentPosition()
        };
    }

    public void goToSlot(int slot) {
        switch (slot) {
            case 1: currentSlotBase = slot1Pos; break;
            case 2: currentSlotBase = slot2Pos; break;
            case 3:
            default: currentSlotBase = slot3Pos; break;
        }
        updateTargets();
    }

    public void setAngle(double offset) {
        angleOffset = Range.clip(offset, minAngle, maxAngle);
        updateTargets();
    }

    private void updateTargets() {
        targetL = currentSlotBase - angleOffset * angleScale;
        targetR = currentSlotBase + angleOffset * angleScale;
    }

    public void reset(double angleOffset) {
        currentSlotBase = 0;
        this.angleOffset = angleOffset;
        targetL = 0;
        targetR = 0;
        resetEncoders();
        diffyL.setPower(0);
        diffyR.setPower(0);
    }

    public void resetEncoders() {
        encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean atTarget() {
        double[] e = getErrors();
        return Math.abs(e[0]) <= toleranceTicks && Math.abs(e[1]) <= toleranceTicks;
    }

    public double aimTurretAngle(double x, double y, double headingRad) {
        if (RobotConfig.alliance == RobotConfig.Alliance.BLUE) {
            double errorX = Math.abs(BLUE_GOAL_X - x);
            double errorY = Math.abs(BLUE_GOAL_Y - y);
            return ((Math.toDegrees(Math.atan2(errorY, errorX))) + Math.toDegrees(headingRad) - 85);
        } else if (RobotConfig.alliance == RobotConfig.Alliance.RED) {
            double errorX = Math.abs(RED_GOAL_X - x);
            double errorY = Math.abs(RED_GOAL_Y - y);
            return ((Math.toDegrees(Math.atan2(errorY, errorX))) + Math.toDegrees(headingRad) - 90);
        } else {
            return 90;
        }
    }
}
