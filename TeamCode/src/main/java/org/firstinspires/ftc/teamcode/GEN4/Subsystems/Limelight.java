package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.concurrent.ArrayBlockingQueue;

public class Limelight {

    private final Limelight3A limelight;
    private final ArrayBlockingQueue<Pose2d> poseWindow;

    // Thresholds
    private static final double METERS_TO_INCHES = 39.37;
    private static final double XY_THRESHOLD = 3; // inches
    private static final double HEADING_THRESHOLD = Math.toRadians(300);

    private final int sampleCount;
    private double lastRelocalizeTime = 0; // in seconds
    private static final double RELOCALIZE_INTERVAL = 1.0; // seconds

    private static final double LL_WEIGHT = 0.01;

    public Limelight(HardwareMap hardwareMap, int sampleCount) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        this.sampleCount = sampleCount;
        poseWindow = new ArrayBlockingQueue<>(sampleCount);
    }

    public Pose2d editAdjustedPose(Pose2d adjustedPose) {
        getLimelightPose();
        double x, y, x2, y2;
        x = adjustedPose.position.x;
        y = adjustedPose.position.y;

        x2 = getLimelightPose().position.x;
        y2 = getLimelightPose().position.x;



        return new Pose2d((x2*LL_WEIGHT) + (1-LL_WEIGHT) * x, (y2*LL_WEIGHT) + (1-LL_WEIGHT) * y, adjustedPose.heading.toDouble());

    }

    public Pose2d getLimelightPose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D pose = result.getBotpose();
            return new Pose2d(
                    pose.getPosition().y * METERS_TO_INCHES,
                    pose.getPosition().x * -METERS_TO_INCHES,
                    pose.getOrientation().getYaw()
            );
        }
        return null;
    }

    /**
     * Call every loop.
     * Returns a stable pose if ready to relocalize, otherwise null.
     */
    public Pose2d tryRelocalize(double currentTime) {
        // Only attempt once per second
        if (currentTime - lastRelocalizeTime < RELOCALIZE_INTERVAL) {
            return null;
        }

        Pose2d llPose = getLimelightPose();
        if (llPose == null) return null;

        // Sliding window of last N poses
        if (poseWindow.remainingCapacity() == 0) {
            poseWindow.poll();
        }
        poseWindow.add(llPose);

        // Not enough samples yet
        if (poseWindow.remainingCapacity() > 0) return null;

        // Check stability
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        double minH = Double.MAX_VALUE, maxH = -Double.MAX_VALUE;

        for (Pose2d p : poseWindow) {
            minX = Math.min(minX, p.position.x);
            maxX = Math.max(maxX, p.position.x);

            minY = Math.min(minY, p.position.y);
            maxY = Math.max(maxY, p.position.y);

            double h = p.heading.toDouble();
            minH = Math.min(minH, h);
            maxH = Math.max(maxH, h);
        }

        boolean stable = (maxX - minX) < XY_THRESHOLD &&
                (maxY - minY) < XY_THRESHOLD &&
                (maxH - minH) < HEADING_THRESHOLD;

        if (stable) {
            // Wait 1 second after relocalization
            lastRelocalizeTime = currentTime;
            poseWindow.clear(); // prevent repeated snaps
            return llPose;
        }

        return null;
    }
}
