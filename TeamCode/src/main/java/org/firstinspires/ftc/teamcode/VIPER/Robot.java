package org.firstinspires.ftc.teamcode.VIPER;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Robot {
    Pose2D targetGoal = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    Turret turret;
    Drivetrain drivetrain;


    /*
    1 = TRACKING + SOTM
    2 = TRACKING
    3 = FIXED
    4 = IDLE
     */


    public void getTargetGoal() {
        if (alliance.isBlue()) {
            targetGoal = new Pose2D(DistanceUnit.INCH, 63, 63, AngleUnit.DEGREES, 0);
        } else {
            targetGoal = new Pose2D(DistanceUnit.INCH, -63, 63, AngleUnit.DEGREES, 0);
        }
    }

    public void update(int turretState) {
        getTargetGoal();
        drivetrain.update();

        if (turretState == 1) {
            turret.state = Turret.State.SOTM;
        } else if (turretState == 2) {
            turret.state = Turret.State.TRACKING;
        } else if (turretState == 3) {
            turret.state = Turret.State.FIXED;
        } else {
            turret.state = Turret.State.IDLE;
        }

        turret.update(drivetrain.robotPose, targetGoal, drivetrain.TVel());

    }




}
