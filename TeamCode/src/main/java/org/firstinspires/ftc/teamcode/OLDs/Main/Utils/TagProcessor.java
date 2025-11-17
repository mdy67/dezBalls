package org.firstinspires.ftc.teamcode.OLDs.Main.Utils;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Simplified AprilTag processor for FTC — detects tag IDs 21, 22, and 23.
 * Includes alliance selection during init (D-pad left = BLUE, D-pad right = RED).
 * Remembers the last detected tag ID during init.
 */
public class TagProcessor {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private int lastDetectedTagId = -1; // stores last valid detection
    private Alliance currentAlliance = Alliance.BLUE;
    public int goToSlot = 3;

    public enum Alliance {
        BLUE, RED
    }

    public TagProcessor(HardwareMap hardwareMap) {
        initAprilTag(hardwareMap);
    }

    private void initAprilTag(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        visionPortal = builder.build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    /** Call during INIT loop — updates detection + allows alliance selection */
    public void handleInitLoop(LinearOpMode opMode) {
        update();

        if (opMode.gamepad1.dpad_left) currentAlliance = Alliance.BLUE;
        else if (opMode.gamepad1.dpad_right) currentAlliance = Alliance.RED;

        opMode.telemetry.addLine("=== APRILTAG / ALLIANCE SELECT ===");
        opMode.telemetry.addData("Alliance", currentAlliance);
        opMode.telemetry.addData("Detected Tag", (lastDetectedTagId == -1) ? "None" : lastDetectedTagId);
        opMode.telemetry.addLine("Use D-Pad: ← = Blue | Red = →");

        opMode.telemetry.update();
    }

    /** Updates detections; keeps last valid tag if nothing new detected */
    public void update() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 21 || detection.id == 22 || detection.id == 23) {
                lastDetectedTagId = detection.id; // update only on valid detection
                break;
            }
        }
    }

    public int getDetectedTagId() {
        return lastDetectedTagId; // always returns last valid detection
    }

    public Alliance getAlliance() {
        return currentAlliance;
    }

    public void close() {
        if (visionPortal != null) visionPortal.close();
    }
}
