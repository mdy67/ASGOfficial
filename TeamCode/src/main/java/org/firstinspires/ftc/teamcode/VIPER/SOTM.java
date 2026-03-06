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
    private double kX = 0.8;
    private double kY = 0.8;
    public Pose2D virtualGoal = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    public void update(Pose2D targetGoal, double XVel, double YVel) {
        calculateVirtualGoal(targetGoal, XVel, YVel);
    }

    private void calculateVirtualGoal(Pose2D targetGoal, double XVel, double YVel) {
        double x = targetGoal.getX(DistanceUnit.INCH) + (XVel * kX);
        double y = targetGoal.getY(DistanceUnit.INCH) + (YVel * kY);

        virtualGoal = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, 0);
    }


}
