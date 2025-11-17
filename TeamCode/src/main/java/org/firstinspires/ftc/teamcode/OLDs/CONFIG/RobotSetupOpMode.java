package org.firstinspires.ftc.teamcode.OLDs.CONFIG;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@TeleOp(name = "Robot Setup", group = "Competitions")
public class RobotSetupOpMode extends OpMode {

    @Override
    public void init() {
        telemetry.addLine("Press UP for RED alliance");
        telemetry.addLine("Press DOWN for BLUE alliance");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            RobotConfig.alliance = RobotConfig.Alliance.RED;
        } else if (gamepad1.dpad_down) {
            RobotConfig.alliance = RobotConfig.Alliance.BLUE;
        }

        telemetry.addData("Selected Alliance", RobotConfig.alliance);
        telemetry.update();
    }
}
