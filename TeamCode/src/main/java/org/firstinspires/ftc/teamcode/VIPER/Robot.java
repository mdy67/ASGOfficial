package org.firstinspires.ftc.teamcode.VIPER;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Robot {
    public HardwareMap hardwareMap;

    public Pose2D targetGoal = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public Turret turret;
    SOTM sotm;
    public Intake intake;
    public Drivetrain drivetrain;
    public Linkage linkage;
    public Shooter shooter;
    public LEDs leds;


    /*
    1 = TRACKING + SOTM
    2 = TRACKING
    3 = FIXED
    4 = IDLE
     */

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        leds = new LEDs(hardwareMap);
        linkage = new Linkage(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        sotm = new SOTM();
    }

    public void getTargetGoal() {
        if (alliance.isBlue()) {
            targetGoal = new Pose2D(DistanceUnit.INCH, -63, 63, AngleUnit.DEGREES, 0);
        } else {
            targetGoal = new Pose2D(DistanceUnit.INCH, 63, 63, AngleUnit.DEGREES, 0);
        }
    }

    public void goToPoint(Pose2D targetPoint, double maxPower, double xyThreshold, double hThreshold) {
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, xyThreshold, hThreshold);
    }

    public void update(int turretState) {
      //  intake.update();
        if (shooter.atVelocity() && !shooter.shooting && drivetrain.inZone()) {
            leds.setOneColor(0.48);
        } else if (shooter.shooting) {
            leds.setOneColor(0.29);
        } else {
            leds.setOneColor(0);
        }
        getTargetGoal();
        drivetrain.update();
        sotm.update(drivetrain.robotPose, targetGoal, drivetrain.XVel(), drivetrain.YVel());
        shooter.update();
        if (turretState == 1) {
            turret.state = Turret.State.SOTM;
            turret.update(drivetrain.robotPose, sotm.virtualGoal, drivetrain.TVel());
            shooter.aimToGoal(sotm.virtualGoal, drivetrain.robotPose);
        } else if (turretState == 2) {
            turret.state = Turret.State.TRACKING;
            turret.update(drivetrain.robotPose, targetGoal, drivetrain.TVel());
            shooter.aimToGoal(targetGoal, drivetrain.robotPose);
        } else if (turretState == 3) {
            turret.state = Turret.State.FIXED;
            turret.update(drivetrain.robotPose, targetGoal, drivetrain.TVel());
        } else {
            turret.state = Turret.State.IDLE;
            turret.update(drivetrain.robotPose, targetGoal, drivetrain.TVel());
        }



    }




}
