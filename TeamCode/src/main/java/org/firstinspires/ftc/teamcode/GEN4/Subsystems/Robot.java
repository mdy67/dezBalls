package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {
    public HardwareMap hardwareMap;

    public Drivetrain drivetrain;
    public Intake intake;
    public Differential differential;
    public Colors colors;
    public Flywheel flywheel;

    public Pose2D blueGoal = new Pose2D(DistanceUnit.INCH, -60, 60, AngleUnit.DEGREES, 0);
    public Pose2D redGoal  = new Pose2D(DistanceUnit.INCH, 60, 60, AngleUnit.DEGREES, 0);

    private boolean flywheelEnabled = false;  // FLAG TO SKIP FLYWHEEL

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);
        differential = new Differential(hardwareMap);

        try {
            flywheel = new Flywheel(hardwareMap, hardwareMap.voltageSensor.iterator().hasNext()
                    ? hardwareMap.voltageSensor.iterator().next() : null);
        } catch (Exception e) {
            flywheel = null;
            flywheelEnabled = false; // flywheel hardware not present
        }
    }

    public void startup() {
        drivetrain.pinpoint.resetPosAndIMU();
        differential.resetEncoders();
    }

    public void update() {
        drivetrain.update();

        if (flywheelEnabled && flywheel != null) {
            flywheel.update();
            differential.update();
        }
    }

    public void goToPoint(Pose2D targetPoint, double maxPower, double xyThreshold, double hThreshold){
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, xyThreshold, hThreshold);
    }

    public void holdPoint(Pose2D targetPoint, double maxPower){
        drivetrain.state = Drivetrain.State.HOLD_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, 0, 0);
    }

    public Pose2D getActiveGoal() {
        if (alliance.isBlue()) return blueGoal;
        else return redGoal;
    }

    public void aimToGoal() {
        if (flywheelEnabled && flywheel != null) {
            Pose2D activeGoal = getActiveGoal();
            differential.aimToGoal(drivetrain.robotPose, drivetrain.XVel(), drivetrain.YVel(), drivetrain.TVel(), activeGoal);
            flywheel.aimToGoal(activeGoal, drivetrain.robotPose, drivetrain.XVel(), drivetrain.YVel());
        }
    }

    public void idleScoringSystems() {
        if (flywheelEnabled && flywheel != null) {
            flywheel.setTargetVelocity(250);
        }
    }

    public void disableFlywheel() {
        flywheelEnabled = false;
        flywheel = null;
    }
}
