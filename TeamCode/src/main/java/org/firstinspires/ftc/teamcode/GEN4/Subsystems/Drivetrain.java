package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GEN4.Misc.DTPID;
import org.firstinspires.ftc.teamcode.GEN4.Misc.Utils;

public class Drivetrain {

    public enum State {
        GO_TO_POINT,
        TELEOP,
        HOLD_POINT,
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
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"); // CONFIG NAMED 12/6/2025
        pinpoint.setOffsets(69.19, -154.75, DistanceUnit.MM); // ACCURATE POS TUNED 12/6/2025
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, // ACCURATE DIRECTION TUNED 12/6/2025
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void setPosition(double x, double y, double heading) { // DEGREES FOR HEADING
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading));
    }

    private void setMotorPowers(double lf, double lb, double rb, double rf) {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
        rightFront.setPower(rf);
    }

    public void setWeightedMotorPowers(double strafe, double fwd, double heading) { // Mecanum movement
        double denominator = Math.max(Math.abs(strafe) + Math.abs(fwd) + Math.abs(heading), 1); // Scaling
        double[] weightPowers = new double[]{
                (fwd - strafe - heading) / denominator,
                (fwd + strafe - heading) / denominator,
                (fwd - strafe + heading) / denominator,
                (fwd + strafe + heading) / denominator
        };
        setMotorPowers(weightPowers[0], weightPowers[1], weightPowers[2], weightPowers[3]);
    }

    // ROBOT POSE
    public Pose2D robotPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 0.01, 0.01, AngleUnit.DEGREES, 0.01);
    private Pose2D lastTarget = targetPose;
    public double targetX, targetY, targetT, maxPower, xyThreshold, hThreshold;

    public double XVel() { return pinpoint.getVelX(DistanceUnit.INCH); }
    public double YVel() { return pinpoint.getVelY(DistanceUnit.INCH); }
    public double TVel() { return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS); }

    public void goToPoint(Pose2D targetPoint, double maxPower, double xyThreshold, double hThreshold) {
        if (targetPoint != lastTarget) {
            targetPose = targetPoint;
            lastTarget = targetPose;

            targetX = targetPose.getX(DistanceUnit.INCH);
            targetY = targetPose.getY(DistanceUnit.INCH);
            targetT = Math.toRadians(targetPose.getHeading(AngleUnit.DEGREES));

            this.maxPower = maxPower;

            this.xyThreshold = xyThreshold;
            this.hThreshold = hThreshold;

        }
    }


    public double tError, xError, yError;
    public double globalXerror, globalYerror;
    public void getErrors() {
        tError = Utils.headingClip(targetT - pinpoint.getHeading(AngleUnit.RADIANS));
        globalXerror = (targetX - pinpoint.getPosX(DistanceUnit.INCH));
        globalYerror = (targetY - pinpoint.getPosY(DistanceUnit.INCH));

        xError = globalXerror *Math.cos(tError) - globalYerror*Math.sin(tError);
        yError = globalYerror*Math.cos(tError) + globalXerror *Math.sin(tError);

    }


    public boolean DTatTarget() {
        return ((Math.abs(xError) + Math.abs(yError)) <= xyThreshold && Math.abs(tError) < hThreshold);
    }

    // ADJUSTED 12/6/2025
    public static final double xkP = 0.1;
    public static final double xkD = 0.01;
    public static final double ykP = 0.1;
    public static final double ykD = 0.01;
    public static final double tkP = 0.1;
    public static final double tkD = 0.01;

    DTPID xPID = new DTPID(xkP,xkD);
    DTPID yPID = new DTPID(ykP, ykD);
    DTPID tPID = new DTPID(tkP, tkD);

    public double xPower, yPower, tPower;
    public void applyPIDPowers() {
        getErrors();

        xPower = xPID.newPDPower(xError, maxPower);
        yPower = yPID.newPDPower(yError, maxPower);
        tPower = tPID.newPDPower(tError, maxPower);

        setWeightedMotorPowers(yPower, xPower, tPower);
    }


    public void update(){
        pinpoint.update();
        // ROBOT POSE DEFINED
        robotPose = pinpoint.getPosition();

        switch (state) {
            case GO_TO_POINT:
                applyPIDPowers();
             //   setWeightedMotorPowers(0, 1, 0);

                if (DTatTarget()) {
                    state = State.IDLE;
                }

                break;

            case TELEOP:
                break;

            case HOLD_POINT:
                applyPIDPowers(); // still goes to TARGET POINT but never exits running PID so it holds pos
                break;

            case IDLE:
                setMotorPowers(0,0,0,0);
                break;

            default:
                break;
        }
    }




}
