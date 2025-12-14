package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {

    public HardwareMap hardwareMap;

    public Drivetrain drivetrain;
    public Differential differential;
    public Flywheel flywheel;
    public Intake intake;
    public Colors colors;
    public Arms arms;
    public Limelight limelight;

    public Pose2D blueGoal = new Pose2D(DistanceUnit.INCH, -62, 65, AngleUnit.DEGREES, 0);
    public Pose2D redGoal  = new Pose2D(DistanceUnit.INCH, 62, 65, AngleUnit.DEGREES, 0);
    public Pose2D adjustedPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    private boolean flywheelEnabled = false;
    public Wait wait = new Wait();

    // =========================
    // RELOCALIZATION VARIABLES
    // =========================
    private static final int RELOCALIZE_SAMPLE_COUNT = 5;
    private double lastRelocalizeTime = 0; // seconds
    private static final double RELOCALIZE_INTERVAL = 1.0; // seconds



    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);
        differential = new Differential(hardwareMap);
        colors = new Colors(hardwareMap);
        arms = new Arms(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = new Limelight(hardwareMap);

        try {
            flywheel = new Flywheel(hardwareMap, hardwareMap.voltageSensor.iterator().hasNext()
                    ? hardwareMap.voltageSensor.iterator().next() : null);
        } catch (Exception e) {
            flywheel = null;
            flywheelEnabled = false;
        }
    }

    public void startup() {
        drivetrain.pinpoint.resetPosAndIMU();
        differential.resetEncoders();
    }

    public void update() {
        drivetrain.update();
        wait.update();
        colors.update();
        flywheel.update();
        differential.update();
        limelight.update();
        adjustedPose = limelight.newAdjustedPose(adjustedPose, drivetrain.lastrobotPose, drivetrain.robotPose, drivetrain.XVel(), drivetrain.YVel(), drivetrain.TVel());

    }

    public void goToPoint(Pose2D targetPoint, double maxPower, double xyThreshold, double hThreshold){
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, xyThreshold, hThreshold);
    }

    public void holdPoint(Pose2D targetPoint, double maxPower){
        drivetrain.state = Drivetrain.State.HOLD_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, 0, 0);
    }

    public Pose2D getTargetGoal() {
        if (org.firstinspires.ftc.teamcode.GEN4.Subsystems.alliance.isRed()) {
            return redGoal;
        } else {
            return blueGoal;
        }
    }

}
