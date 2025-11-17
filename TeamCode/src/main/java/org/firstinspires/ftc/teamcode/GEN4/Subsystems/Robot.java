package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Robot {
    public Drivetrain drivetrain;
    drivetrain = new Drivetrain(HardwareMap);

    public void update() {

    }

    public void goToPoint(Pose2D targetPoint, boolean brake, boolean finalAdjust, double maxPower, double moveOnThreshold){

    }
}
