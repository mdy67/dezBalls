package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {
    public HardwareMap hardwareMap;

    public Drivetrain drivetrain;
    public Differential differential;
    public Flywheel flywheel;
    public Wait wait = new Wait(); // Wait helper

    private boolean flywheelEnabled = false;

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);
        differential = new Differential(hardwareMap);

        try {
            flywheel = new Flywheel(hardwareMap,
                    hardwareMap.voltageSensor.iterator().hasNext()
                            ? hardwareMap.voltageSensor.iterator().next()
                            : null);
        } catch (Exception e) {
            flywheel = null;
            flywheelEnabled = false;
        }
    }

    public void startup() {
        drivetrain.pinpoint.resetPosAndIMU();
        differential.resetEncoders();
    }

    public void update() {
        drivetrain.update();
        wait.update(); // critical: update wait timer each loop

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

    public void disableFlywheel() {
        flywheelEnabled = false;
        flywheel = null;
    }
}
