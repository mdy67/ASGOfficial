package org.firstinspires.ftc.teamcode.VIPER;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

    private ElapsedTime rapidFireTimer = new ElapsedTime();
    private boolean rapidFireRunning = false;
    private double kTime = 0.41;

    private ElapsedTime autoFireTimer = new ElapsedTime();
    private boolean autoFireRunning = false;
    private boolean autoFireDone = false;

    private double autoFireMaxTime = 1.0;

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
            targetGoal = new Pose2D(DistanceUnit.INCH, -65, 72, AngleUnit.DEGREES, 0);
        } else {
            targetGoal = new Pose2D(DistanceUnit.INCH, 65, 72, AngleUnit.DEGREES, 0);
        }
    }

    public void rapidFire() {
        if (!rapidFireRunning) {
            rapidFireTimer.reset();
            rapidFireRunning = true;
        }

        linkage.OFF();
        shooter.shooting = true;

        double power = Range.clip(
                -1 + (rapidFireTimer.seconds() * kTime),
                -1,
                -0.76
        );

        intake.setPower(power);
    }

    public void stopRapidFire() {
        rapidFireRunning = false;
        shooter.shooting = false;
        intake.setPower(0);
        linkage.ON();
    }

    // 🔴 FIXED AUTO FIRE
    public void autoRapidFire() {

        if (autoFireDone) return;

        if (!autoFireRunning) {
            autoFireTimer.reset();
            autoFireRunning = true;
        }

        rapidFire();

        if (autoFireTimer.seconds() > autoFireMaxTime) {
            stopRapidFire();
            autoFireRunning = false;
            shooter.shooting = false;
            autoFireDone = true;
        }
    }

    public boolean autoRapidFireDone() {
        return autoFireDone;
    }

    public void resetAutoRapidFire() {
        autoFireRunning = false;
        autoFireDone = false;
    }

    public void goToPoint(Pose2D targetPoint, double maxPower, double xyThreshold, double hThreshold) {
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, xyThreshold, hThreshold);
    }

    public void update(int turretState) {
        if (shooter.atVelocity() && !shooter.shooting && drivetrain.inZone()) {
            if (alliance.isBlue()) {
                leds.setOneColor(0.611);
            } else {
                leds.setOneColor(0.28);
            }

        } else if (shooter.shooting) {
            leds.setOneColor(0.500);
        } else {
            leds.setOneColor(0);
        }

        turret.HEIGHT_OFFSET = ((double) 2 / 72) * drivetrain.robotPose.getY(DistanceUnit.INCH);

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
            shooter.aimToGoal(targetGoal, drivetrain.robotPose);
        } else {
            turret.state = Turret.State.IDLE;
            shooter.stop();
            turret.update(drivetrain.robotPose, targetGoal, drivetrain.TVel());
          //  shooter.aimToGoal(targetGoal, drivetrain.robotPose);
        }
    }
}