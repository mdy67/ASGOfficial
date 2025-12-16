package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {

    private final Limelight3A limelight;

    private static final double METERS_TO_INCHES = 39.37;
    private static final double LL_WEIGHT = 0.1;
    private static final double XYThreshold = 1;
    private static final double TThreshold = 2;

    public Pose2d LimelightPose = new Pose2d(0, 0, 0);
    public Pose2d LastLimelightPose = new Pose2d(0, 0, 0);

    private static final double BadPoseThreshold = 24; // inches

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void update() {
        // Only advance last pose if current pose is valid
        if (LimelightPose != null) {
            LastLimelightPose = LimelightPose;
        }
        LimelightPose = getLimelightPose();
    }

    public boolean BadPose() {
        if (LimelightPose == null || LastLimelightPose == null) {
            return true;
        }

        double dx = Math.abs(LimelightPose.position.x - LastLimelightPose.position.x);
        double dy = Math.abs(LimelightPose.position.y - LastLimelightPose.position.y);

        return dx > BadPoseThreshold || dy > BadPoseThreshold;
    }

    public Pose2D newAdjustedPose(
            Pose2D adjustedPose,
            Pose2D lastPose,
            Pose2D currentPose,
            double xVel,
            double yVel,
            double tVel
    ) {
        if (adjustedPose == null || lastPose == null || currentPose == null) {
            return adjustedPose;
        }

        boolean canFuse =
                LimelightPose != null &&
                        xVel < XYThreshold &&
                        yVel < XYThreshold &&
                        tVel < TThreshold &&
                        !BadPose();

        if (canFuse) {
            double x = adjustedPose.getX(DistanceUnit.INCH);
            double y = adjustedPose.getY(DistanceUnit.INCH);

            double x2 = LimelightPose.position.x;
            double y2 = LimelightPose.position.y;

            return new Pose2D(
                    DistanceUnit.INCH,
                    (x2 * LL_WEIGHT) + ((1 - LL_WEIGHT) * x),
                    (y2 * LL_WEIGHT) + ((1 - LL_WEIGHT) * y),
                    AngleUnit.DEGREES,
                    currentPose.getHeading(AngleUnit.DEGREES)
            );
        }

        // Odometry fallback
        double dx = currentPose.getX(DistanceUnit.INCH) - lastPose.getX(DistanceUnit.INCH);
        double dy = currentPose.getY(DistanceUnit.INCH) - lastPose.getY(DistanceUnit.INCH);

        return new Pose2D(
                DistanceUnit.INCH,
                adjustedPose.getX(DistanceUnit.INCH) + dx,
                adjustedPose.getY(DistanceUnit.INCH) + dy,
                AngleUnit.DEGREES,
                currentPose.getHeading(AngleUnit.DEGREES)
        );
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

        // NEVER return null
        return LastLimelightPose;
    }
}
