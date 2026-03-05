package org.firstinspires.ftc.teamcode.GEN4.OLDsubsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Limelight {

    private final Limelight3A limelight;

    private static final double METERS_TO_INCHES = 39.37;

    public Pose2d LimelightPose = new Pose2d(0,0,0);
    public Pose2d LastLimelightPose = new Pose2d(0,0,0);

    // ARBITRARY OFFSETS FOR ACCURACY TODO: TUNE
    public static final double X_OFFSET = -6;
    public static final double Y_OFFSET = -3;

    private int tagId = 0;

    public boolean tagDetected = false;

    private List<LLResultTypes.FiducialResult> tags;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (!fiducials.isEmpty()) {
            tagId = fiducials.get(0).getFiducialId();
        }

        if (result.isValid()) {
            Pose3D pose = result.getBotpose();

            // Only update if the pose is non-zero
            double xInches = pose.getPosition().y * METERS_TO_INCHES;
            double yInches = -pose.getPosition().x * METERS_TO_INCHES;

            if (xInches != 0.0 && yInches != 0.0) {
                LimelightPose = new Pose2d(
                        xInches,
                        yInches,
                        pose.getOrientation().getYaw()
                );
                LastLimelightPose = LimelightPose;
                tagDetected = true;
            } else {
                // invalid pose, don't fuse
                tagDetected = false;
                LimelightPose = LastLimelightPose;
            }

        } else {
            tagDetected = false;
            // keep last known pose
            LimelightPose = LastLimelightPose;
        }
    }

    public Pose2d getPose() {
        return LimelightPose;
    }

    /**
     * Returns the detected AprilTag ID (21, 22, 23), or 0 if none detected.
     */
    public int getMotif() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            if (tagId == 21 || tagId == 22 || tagId == 23) {
                return tagId;
            }

        }
        return 0; // No valid tag detected
    }
}
