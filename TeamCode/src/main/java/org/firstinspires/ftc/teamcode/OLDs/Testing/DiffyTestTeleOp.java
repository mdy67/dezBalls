package org.firstinspires.ftc.teamcode.OLDs.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems.Diffy;

@TeleOp(name = "Diffy Slot+Angle Test", group = "Tests")
public class DiffyTestTeleOp extends OpMode {

    private Diffy diffy;
    private FtcDashboard dashboard;

    private int currentSlot = 3;
    private double turnOffset = 0;

    @Override
    public void init() {
        diffy = new Diffy(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        diffy.reset(90);
        telemetry.addLine("Diffy Slot + Angle Test Ready");
        telemetry.addLine("A/B/X = Slot 1/2/3");
        telemetry.addLine("Left Stick X = Rotate turret");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Slot switching ---
        if (gamepad1.a) currentSlot = 1;
        else if (gamepad1.b) currentSlot = 2;
        else if (gamepad1.x) currentSlot = 3;

        diffy.goToSlot(currentSlot);

        // --- Angle control ---
        turnOffset += gamepad1.left_stick_x * 5; // scale turn speed
        diffy.setAngle(turnOffset);

        // --- Update diffy ---
        diffy.update();

        // --- Dashboard telemetry ---
        double[] e = diffy.getErrors();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Slot", currentSlot);
        packet.put("Turn Offset", turnOffset);
        packet.put("Target L", Diffy.targetL);
        packet.put("Target R", Diffy.targetR);
        packet.put("Error L", e[0]);
        packet.put("Error R", e[1]);
        packet.put("Encoder L", diffy.encL.getCurrentPosition());
        packet.put("Encoder R", diffy.encR.getCurrentPosition());
        packet.put("Power L", diffy.diffyL.getPower());
        packet.put("Power R", diffy.diffyR.getPower());
        packet.put("At Target", diffy.atTarget());
        dashboard.sendTelemetryPacket(packet);

        // --- Standard telemetry ---
        telemetry.addData("Slot", currentSlot);
        telemetry.addData("Turn Offset", turnOffset);
        telemetry.addData("TargetL", Diffy.targetL);
        telemetry.addData("TargetR", Diffy.targetR);
        telemetry.addData("ErrorL", e[0]);
        telemetry.addData("ErrorR", e[1]);
        telemetry.addData("At Target", diffy.atTarget());
        telemetry.update();
    }
}
