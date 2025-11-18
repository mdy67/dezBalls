package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Robot {
    public HardwareMap hardwareMap;

    public Drivetrain drivetrain;

    public void update() {

    }

    public void goToPoint(Pose2D targetPoint, boolean brake, boolean finalAdjust, double maxPower, double xyThreshold, double hThreshold){
        drivetrain.goToPoint(targetPoint, brake, finalAdjust, maxPower, xyThreshold, hThreshold);
    }
}
