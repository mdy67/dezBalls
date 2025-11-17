package org.firstinspires.ftc.teamcode.OLDs.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.OLDs.Main.Robot;

@TeleOp(name = "ENCODER TEST", group = "Testing")
public class EncoderTest extends OpMode {

    Robot robot;
    private DcMotorEx intake, leftBack, flywheelR;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        // Initialize all arms down
        robot.arms.reset();

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        flywheelR = hardwareMap.get(DcMotorEx.class, "shooterL");
    }

    @Override
    public void loop() {
        telemetry.addData("REVCoder L Pos:", intake.getCurrentPosition());
        telemetry.addData("REVCoder R Pos:", leftBack.getCurrentPosition());
        telemetry.addData("FLYWHEEL Pos:", flywheelR.getCurrentPosition());
        telemetry.update();
    }
}
