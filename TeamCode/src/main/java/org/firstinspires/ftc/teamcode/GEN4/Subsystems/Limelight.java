package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.concurrent.ArrayBlockingQueue;

public class Limelight {

    private final Limelight3A limelight;

    // Thresholds
    private static final double METERS_TO_INCHES = 39.37;
    private static final double LL_WEIGHT = 0.1;
    private static final double XYThreshold = 1;
    private static final double TThreshold = 2;

    public Pose2d LimelightPose = new Pose2d(0, 0, 0);
    public Pose2d LastLimelightPose = new Pose2d(0, 0, 0);

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

    }
    double x, y, t, x2, y2, t2;

    public Pose2D newAdjustedPose(Pose2D adjustedPose, Pose2D lastPose, Pose2D CurrentPose, double xVel, double yVel, double tVel) {
        if (LimelightPose != null
                && xVel < XYThreshold
                && yVel < XYThreshold
                && tVel < TThreshold
                && !BadPose() ) {
            if (LimelightPose.position.x * LimelightPose.position.y * LimelightPose.heading.toDouble() != 0) {
                double x, y, x2, y2;
                x = adjustedPose.getX(DistanceUnit.INCH);
                y = adjustedPose.getY(DistanceUnit.INCH);

                x2 = LimelightPose.position.x;
                y2 = LimelightPose.position.y;

                double LL_WEIGHT_X = LL_WEIGHT;
                double LL_WEIGHT_Y = LL_WEIGHT;
                if (Math.abs(xVel) > XYThreshold) {
                    LL_WEIGHT_X = Math.abs(LL_WEIGHT * (XYThreshold / xVel));
                }
                if (Math.abs(yVel) > XYThreshold) {
                    LL_WEIGHT_Y = Math.abs(LL_WEIGHT * (XYThreshold / yVel));
                }


                return new Pose2D(DistanceUnit.INCH, (x2*LL_WEIGHT_X) + ((1-LL_WEIGHT_X) * x), (y2*LL_WEIGHT_Y) + ((1-LL_WEIGHT_Y) * y), AngleUnit.DEGREES, CurrentPose.getHeading(AngleUnit.DEGREES));
            } else {
                x = CurrentPose.getX(DistanceUnit.INCH) - lastPose.getX(DistanceUnit.INCH);
                y = CurrentPose.getY(DistanceUnit.INCH) - lastPose.getY(DistanceUnit.INCH);

                x2 = adjustedPose.getX(DistanceUnit.INCH);
                y2 = adjustedPose.getY(DistanceUnit.INCH);
                t2 = adjustedPose.getHeading(AngleUnit.DEGREES);
                return new Pose2D(DistanceUnit.INCH, x+x2, y+y2, AngleUnit.DEGREES, CurrentPose.getHeading(AngleUnit.DEGREES));
            }

        } else {
            x = CurrentPose.getX(DistanceUnit.INCH) - lastPose.getX(DistanceUnit.INCH);
            y = CurrentPose.getY(DistanceUnit.INCH) - lastPose.getY(DistanceUnit.INCH);

            x2 = adjustedPose.getX(DistanceUnit.INCH);
            y2 = adjustedPose.getY(DistanceUnit.INCH);
            t2 = adjustedPose.getHeading(AngleUnit.DEGREES);
            return new Pose2D(DistanceUnit.INCH, x+x2, y+y2, AngleUnit.DEGREES, CurrentPose.getHeading(AngleUnit.DEGREES));
        }
    }

    public void update() {
        LastLimelightPose = LimelightPose;
        LimelightPose = getLimelightPose();

    }

    double BadPoseMultiplier = 2;
    public boolean BadPose() {
        if (LimelightPose != null) {
            return LimelightPose.position.x > LastLimelightPose.position.x * BadPoseMultiplier
                    || LimelightPose.position.y > LastLimelightPose.position.y * BadPoseMultiplier;
        } else {
            return true;
        }

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

}
