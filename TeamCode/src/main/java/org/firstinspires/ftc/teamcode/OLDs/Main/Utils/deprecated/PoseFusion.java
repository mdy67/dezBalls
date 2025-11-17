package org.firstinspires.ftc.teamcode.OLDs.Main.Utils.deprecated;

import com.acmerobotics.roadrunner.Pose2d;
import java.util.LinkedList;
import java.util.Queue;

public class PoseFusion {

    private Pose2d fusedPose = new Pose2d(0, 0, Math.toRadians(90));
    private Pose2d lastOdom = null;

    private double tagWeight = 0.0005;  // gentle nudge
    private double maxTagJumpInch = 0.5;
    private int consistencyWindow = 40;   // number of consecutive readings to consider
    private double maxTagVariance = 0.5; // inches

    private final Queue<Pose2d> recentTagPoses = new LinkedList<>();

    public PoseFusion() {}

    public PoseFusion(double tagWeight, double maxTagJumpInch) {
        this.tagWeight = tagWeight;
        this.maxTagJumpInch = maxTagJumpInch;
    }

    /**
     * Update fused pose using odometry and AprilTag pose.
     * Only X/Y are fused; heading is ignored.
     */
    public Pose2d update(Pose2d odomPose, Pose2d tagPose) {
        if (lastOdom == null) {
            lastOdom = odomPose;
            fusedPose = odomPose;
            return fusedPose;
        }

        // Apply odometry delta
        double dxOdo = odomPose.position.x - lastOdom.position.x;
        double dyOdo = odomPose.position.y - lastOdom.position.y;

        double fusedX = fusedPose.position.x + dxOdo;
        double fusedY = fusedPose.position.y + dyOdo;

        // Handle tag
        if (tagPose != null && (tagPose.position.x != 0 || tagPose.position.y != 0)) {
            // Add current tag reading to the queue
            recentTagPoses.add(tagPose);
            if (recentTagPoses.size() > consistencyWindow) {
                recentTagPoses.poll();
            }

            // Check variance of the recent readings
            if (isTagConsistent()) {
                double avgX = 0;
                double avgY = 0;
                for (Pose2d p : recentTagPoses) {
                    avgX += p.position.x;
                    avgY += p.position.y;
                }
                avgX /= recentTagPoses.size();
                avgY /= recentTagPoses.size();

                // Compute delta and limit jump
                double dxTag = avgX - fusedX;
                double dyTag = avgY - fusedY;
                double dist = Math.hypot(dxTag, dyTag);

                if (dist > maxTagJumpInch) {
                    double scale = maxTagJumpInch / dist;
                    dxTag *= scale;
                    dyTag *= scale;
                }

                fusedX += dxTag * tagWeight;
                fusedY += dyTag * tagWeight;
            }
        } else {
            // No tag detected, clear history
            recentTagPoses.clear();
        }

        fusedPose = new Pose2d(fusedX, fusedY, fusedPose.heading.toDouble());
        lastOdom = odomPose;
        return fusedPose;
    }

    /** Checks if the recent tag readings are consistent enough */
    private boolean isTagConsistent() {
        if (recentTagPoses.size() < consistencyWindow) return false;

        double avgX = 0, avgY = 0;
        for (Pose2d p : recentTagPoses) {
            avgX += p.position.x;
            avgY += p.position.y;
        }
        avgX /= recentTagPoses.size();
        avgY /= recentTagPoses.size();

        for (Pose2d p : recentTagPoses) {
            double dx = p.position.x - avgX;
            double dy = p.position.y - avgY;
            if (Math.hypot(dx, dy) > maxTagVariance) return false;
        }
        return true;
    }

    public Pose2d getFusedPose() {
        return fusedPose;
    }
}
