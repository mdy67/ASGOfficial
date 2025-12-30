package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class AutoToTeleop {
    public static double encLOffset;
    public static double encROffset;
    public static Pose2D storedPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public double x = storedPose.getX(DistanceUnit.INCH);
    public double y = storedPose.getY(DistanceUnit.INCH);
    public double heading = storedPose.getHeading(AngleUnit.DEGREES);
}
