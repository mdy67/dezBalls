package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {
    public HardwareMap hardwareMap;

    public Drivetrain drivetrain;
    public Intake intake;
    public Differential differential;
    public Colors colors;
    public Flywheel flywheel;

    // TODO: ADJUST GOAL POSITIONS
    public Pose2D blueGoal = new Pose2D(DistanceUnit.INCH, -60, 60, AngleUnit.DEGREES, 0);
    public Pose2D redGoal  = new Pose2D(DistanceUnit.INCH, 60, 60, AngleUnit.DEGREES, 0);

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

        drivetrain = new Drivetrain(hardwareMap);
        differential = new Differential(hardwareMap);
        flywheel = new Flywheel(hardwareMap, battery);
    }

    public void startup() {
        drivetrain.pinpoint.resetPosAndIMU();
        differential.resetEncoders();
    }

    public void update() {
        if (colors.hasBall()) {
            aimToGoal();
        } else {
            idleScoringSystems();
        }

        drivetrain.update();
        intake.update();
        differential.update();
        colors.update();
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
        Pose2D activeGoal = getActiveGoal();
        differential.aimToGoal(drivetrain.robotPose, drivetrain.XVel(), drivetrain.YVel(), drivetrain.TVel(), activeGoal);
        flywheel.aimToGoal(activeGoal, drivetrain.robotPose, drivetrain.XVel(), drivetrain.YVel());
    }

    public void idleScoringSystems() {
        flywheel.setTargetVelocity(250); // IDLE FLYWHEEL VELOCITY
    }
}
