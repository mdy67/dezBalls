package org.firstinspires.ftc.teamcode.OLDs.Main.Utils;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "SERVO COMMANDER", group = "Testing")
public class Servo0Commander extends OpMode {
    private Servo servo0;

    private double save1, save2;


    private double tgtPos;

    @Override public void init() {
        servo0 = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        if (Math.abs(gamepad1.right_stick_y) > 0.1f) {
            tgtPos += gamepad1.right_stick_y * 0.001;
        }

        if (gamepad1.dpad_down) {
            save1 = tgtPos;
        } else if (gamepad1.dpad_up) {
            save2 = tgtPos;
        }
        if (gamepad1.a) {
            tgtPos = save1;
        } else if (gamepad1.b) {
            tgtPos = save2;
        }

        servo0.setPosition(tgtPos);
        //   rhinoR.setPower(tgtPos);
        telemetry.addData("SERVO POS:", tgtPos);
        telemetry.addData("Pos 1:", save1);
        telemetry.addData("Pos 2:", save2);
        telemetry.addLine();
        if (tgtPos == save1) {
            telemetry.addData("POSITION: 1", "");
        } else if (tgtPos == save2) {
            telemetry.addData("POSITION: 2", "");
        } else {
            telemetry.addData("POSITION: FLOATING", "");
        }


        telemetry.update();
    }
}
