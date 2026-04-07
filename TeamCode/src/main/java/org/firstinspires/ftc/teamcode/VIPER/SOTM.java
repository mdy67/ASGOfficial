package org.firstinspires.ftc.teamcode.VIPER;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class SOTM {
    /**
     *
     * Change in X/Y position of the goal = Axial Velocity *  kM * TOF (TOF IS OPTIONAL)
     *
     * @param robotPos
     * @param targetGoal
     * @param XVel
     * @param YVel
     */
    private double kXClose = 1.4;
    private double kYClose = 1.4;
    private double kXFar = 1.2;
    private double kYFar = 1.2;

    private double farZoneMax = -24; // above this it will be close zone SOTM


    public Pose2D virtualGoal = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    public void update(Pose2D robotPose, Pose2D targetGoal, double XVel, double YVel) {
        calculateVirtualGoal(robotPose, targetGoal, XVel, YVel);
    }

    private void calculateVirtualGoal(Pose2D robotPose, Pose2D targetGoal, double XVel, double YVel) {
        double x = targetGoal.getX(DistanceUnit.INCH) + (XVel * robotPose.getY(DistanceUnit.INCH) > farZoneMax ? kXClose : kXFar);
        double y = targetGoal.getY(DistanceUnit.INCH) + (YVel * robotPose.getY(DistanceUnit.INCH) > farZoneMax ? kYClose : kYFar);

        virtualGoal = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, 0);
    }


}
