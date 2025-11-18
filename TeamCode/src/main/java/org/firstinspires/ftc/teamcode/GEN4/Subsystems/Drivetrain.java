package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Misc.DTPID;
import org.firstinspires.ftc.teamcode.GEN4.Misc.Utils;

public class Drivetrain {

    public enum State {
        GO_TO_POINT,
        TELEOP,
        BRAKE,
        HOLD_POINT,
        FINAL_ADJUSTMENT,
        IDLE
    }
    public State state = State.IDLE;

    public DcMotor leftFront, leftBack, rightBack, rightFront;
    GoBildaPinpointDriver pinpoint;

    public Drivetrain(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    private void setMotorPowers(double lf, double lb, double rb, double rf) {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
        rightFront.setPower(rf);
    }

    public void setWeightedMotorPowers(double strafe, double fwd, double heading) { // Mecanaum movement
        double denominator = Math.max(Math.abs(strafe) + Math.abs(fwd) + Math.abs(heading), 1); // Scaling
        double[] weightPowers = new double[]{
                (fwd + strafe + heading) / denominator,
                (fwd - strafe + heading) / denominator,
                (fwd - strafe - heading) / denominator,
                (fwd + strafe - heading) / denominator
        };
        setMotorPowers(weightPowers[0], weightPowers[1], weightPowers[2], weightPowers[3]);
    }

    public Pose2D robotPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 0.01, 0.01, AngleUnit.DEGREES, 0.01);
    private Pose2D lastTarget = targetPose;
    public double targetX, targetY, targetT, maxPower, xyThreshold, hThreshold;
    public boolean brake, finalAdjust;

    public void goToPoint(Pose2D targetPoint, boolean brake, boolean finalAdjust, double maxPower, double xyThreshold, double hThreshold) {
        if (targetPoint != lastTarget) {
            targetPose = targetPoint;
            lastTarget = targetPose;

            targetX = targetPose.getX(DistanceUnit.INCH);
            targetY = targetPose.getY(DistanceUnit.INCH);
            targetT = targetPose.getHeading(AngleUnit.DEGREES);

            this.brake = brake;
            this.finalAdjust = finalAdjust;
            this.maxPower = maxPower;

            this.xyThreshold = xyThreshold;
            this.hThreshold = hThreshold;

            state = State.GO_TO_POINT;
        }
    }

    public double tError, xError, yError;
    public void getErrors() {
        tError = Utils.headingClip(targetT - pinpoint.getHeading(AngleUnit.DEGREES));
        xError = Utils.headingClip(targetX - pinpoint.getPosX(DistanceUnit.INCH));
        yError = Utils.headingClip(targetY - pinpoint.getPosY(DistanceUnit.INCH));
    }

    public boolean chassisAtTarget() {
        return ((Math.abs(xError) + Math.abs(yError)) <= xyThreshold && Math.abs(tError) < hThreshold);
    }

    DTPID xPID = new DTPID(0,0);
    DTPID yPID = new DTPID(0, 0);
    DTPID tPID = new DTPID(0, 0);

    public void applyPIDPowers() {
        getErrors();

        double xPower = xPID.newPDPower(xError, -maxPower, maxPower);
        double yPower = yPID.newPDPower(yError, -maxPower, maxPower);
        double tPower = tPID.newPDPower(tError, -maxPower, maxPower);

        setWeightedMotorPowers(yPower, xPower, tPower);
    }

    public void update(){
        pinpoint.update();
        robotPose = pinpoint.getPosition();

        switch (state) {
            case GO_TO_POINT:
                applyPIDPowers();

                if (chassisAtTarget()) {
                    if (finalAdjust) {
                        state - State.FINAL_ADJUSTMENT;
                    } else if () {
                        state = State.IDLE;
                    }
                }

                break;

            case TELEOP:
                break;

            case HOLD_POINT:
                break;

            case FINAL_ADJUSTMENT:
                break;

            case IDLE:
                break;


            default:
                // code if no cases match
                break;
        }
    }




}
