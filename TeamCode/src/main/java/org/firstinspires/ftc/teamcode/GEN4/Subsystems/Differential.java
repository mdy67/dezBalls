package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Differential {

    public static double targetL = 0;
    public static double targetR = 0;

    public static final double kPL = 0.00028;
    public static final double kPr = 0.00028;

    public static double minPower = 0;
    public static double maxPower = 1.0;
    public static double toleranceTicks = 175;

    public static final double MIN_ANGLE = 0;
    public static final double MAX_ANGLE = 180;

    public static double slot1Pos = -5300;
    public static double slot2Pos = -2300;
    public static double slot3Pos = 0;

    public static double angleOffset = 0;
    public static double angleScale = 34.4;
    private double currentSlotBase = 0;

    public static double turretOffset = -8.0;

    public CRServo diffyL, diffyR;
    public DcMotorEx encL, encR;

    public Differential(HardwareMap hardwareMap) {
        diffyL = hardwareMap.get(CRServo.class, "diffyL");
        diffyR = hardwareMap.get(CRServo.class, "diffyR");

        encL = hardwareMap.get(DcMotorEx.class, "intakeL");
        encR = hardwareMap.get(DcMotorEx.class, "leftBack");

        encL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        diffyL.setDirection(CRServo.Direction.FORWARD);
        diffyR.setDirection(CRServo.Direction.REVERSE);

        resetEncoders();
    }

    public void update() {
        applyPowers();
    }

    public void resetEncoders() {
        encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void applyPowers() {
        double[] powers = targetPowers();
        diffyL.setPower(powers[0]);
        diffyR.setPower(powers[1]);
    }

    private double[] targetPowers() {
        double[] errors = getErrors();
        double leftPower  = Range.clip(errors[0] * kPL, -maxPower, maxPower);
        double rightPower = Range.clip(errors[1] * kPr, -maxPower, maxPower);
        return new double[]{ leftPower, rightPower };
    }

    private double[] getErrors() {
        return new double[]{
                targetL - encL.getCurrentPosition(),
                targetR - encR.getCurrentPosition()
        };
    }

    public void goToSlot(int slot){
        switch(slot) {
            case 1: currentSlotBase = slot1Pos; break;
            case 2: currentSlotBase = slot2Pos; break;
            case 3:
            default: currentSlotBase = slot3Pos; break;
        }
    }

    public void setTargetAngle(double targetAngle){
        angleOffset = Range.clip(targetAngle, MIN_ANGLE, MAX_ANGLE);
        targetL = currentSlotBase - (angleOffset * angleScale);
        targetR = currentSlotBase + (angleOffset * angleScale);
    }

    public void aimToGoal(Pose2D currentPose, double xVelocity, double yVelocity, double tVelocity, Pose2D targetGoal) {
        double heading = currentPose.getHeading(AngleUnit.RADIANS);

        double kVelT = 0.5, kVelX = 0.5, kVelY = 0.5;
        double worldOffsetX = Math.cos(heading + (tVelocity * kVelT)) * turretOffset;
        double worldOffsetY = Math.sin(heading + (tVelocity * kVelT)) * turretOffset;

        double turretX = currentPose.getX(DistanceUnit.INCH) + worldOffsetX;
        double turretY = currentPose.getY(DistanceUnit.INCH) + worldOffsetY;

        double dx = (targetGoal.getX(DistanceUnit.INCH) - turretX) + (xVelocity * kVelX);
        double dy = (targetGoal.getY(DistanceUnit.INCH) - turretY) + (yVelocity * kVelY);

        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        setTargetAngle(targetAngleDeg);
    }
}
