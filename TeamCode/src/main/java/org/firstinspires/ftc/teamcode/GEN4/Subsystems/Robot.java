package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems.Arms;
import org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems.Diffy;
import org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems.Spatula;
import org.firstinspires.ftc.teamcode.OLDs.RRs.MecanumDrive;

public class Robot {
    public HardwareMap hardwareMap;

    public Drivetrain drivetrain;
    public Intake intake;
    public Differential differential;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
        differential = new Differential(hardwareMap);

        // --- Grab the first voltage sensor in the hardware map ---
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

    }

    public void startup() {
        drivetrain.pinpoint.resetPosAndIMU();
        differential.resetDifferential();
    }

    // TODO: ===== UPDATE ALL SUBSYSTEMS =====
    public void update() { // update ALL subsystems this time no trolling
        drivetrain.update();
        intake.update();
    }

    public void goToPoint(Pose2D targetPoint, double maxPower, double xyThreshold, double hThreshold){
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, xyThreshold, hThreshold);
    }

    public void holdPoint(Pose2D targetPoint, double maxPower){ // thresholds can be null
        drivetrain.state = Drivetrain.State.HOLD_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, 0, 0);
    }



}
