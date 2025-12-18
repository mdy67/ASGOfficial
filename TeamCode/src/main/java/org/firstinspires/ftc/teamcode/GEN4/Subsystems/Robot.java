package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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

    // This is the pose used by all targeting, adjusted with Limelight and odometry
    public Pose2D adjustedPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public Wait wait = new Wait();

    public double MotifTagID = 21; // 21 BY DEFAULT

    // =========================
    // ODOMETRY + LIMELIGHT OFFSET
    // =========================
    private Pose2D odomOffset = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private boolean offsetInitialized = false;

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
        }
    }

    public void startup() {
        arms.arm3_flickOFF();
        arms.arm2_flickOFF();
        arms.arm1_flickOFF();
        drivetrain.pinpoint.resetPosAndIMU();
        differential.resetEncoders();
    }

    // =========================
    // MAIN UPDATE LOOP
    // =========================
    public void update() {
        drivetrain.update();
        wait.update();
        colors.update();
        flywheel.update();
        differential.update();
        limelight.update();

        // ------------------------
        // Corrected Pose Fusion
        // ------------------------
        // Use Pose2D from drivetrain
        Pose2D odomPose = drivetrain.robotPose;

        // Only use Limelight if a tag is detected and pose is non-zero
        boolean hasValidTag = limelight.LimelightPose != null
                && limelight.LimelightPose.position.x != 0.0
                && limelight.LimelightPose.position.y != 0.0
                && limelight.tagDetected;

        // If Limelight is valid & velocity is low, update offset
        boolean canFuse = hasValidTag
                && drivetrain.XVel() < 1.0
                && drivetrain.YVel() < 1.0
                && drivetrain.TVel() < 2.0;

        if (canFuse) {
            if (!offsetInitialized) {
                // Initialize offset
                odomOffset = new Pose2D(DistanceUnit.INCH,
                        limelight.LimelightPose.position.x - odomPose.getX(DistanceUnit.INCH),
                        limelight.LimelightPose.position.y - odomPose.getY(DistanceUnit.INCH),
                        AngleUnit.DEGREES,
                        0);
                offsetInitialized = true;
            } else {
                // Smoothly blend offsets (0.1 weight)
                double newOffsetX = odomOffset.getX(DistanceUnit.INCH) * (1 - 0.1)
                        + (limelight.LimelightPose.position.x - odomPose.getX(DistanceUnit.INCH)) * 0.1;
                double newOffsetY = odomOffset.getY(DistanceUnit.INCH) * (1 - 0.1)
                        + (limelight.LimelightPose.position.y - odomPose.getY(DistanceUnit.INCH)) * 0.1;

                odomOffset = new Pose2D(DistanceUnit.INCH, newOffsetX, newOffsetY, AngleUnit.DEGREES, 0);
            }
        }

        // Always compute adjusted pose (odometry + offset)
        adjustedPose = new Pose2D(
                DistanceUnit.INCH,
                odomPose.getX(DistanceUnit.INCH) + odomOffset.getX(DistanceUnit.INCH) + Limelight.X_OFFSET,
                odomPose.getY(DistanceUnit.INCH) + odomOffset.getY(DistanceUnit.INCH) + Limelight.Y_OFFSET,
                AngleUnit.DEGREES,
                odomPose.getHeading(AngleUnit.DEGREES)
        );
    }

    // =========================
    // DRIVETRAIN / TARGETING METHODS
    // =========================
    public void goToPoint(Pose2D targetPoint, double maxPower, double xyThreshold, double hThreshold){
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, xyThreshold, hThreshold);
    }

    public void goalLock(Pose2D currentPose) {
        differential.aimToGoal(currentPose, getTargetGoal());
        flywheel.aimToGoal(getTargetGoal(), currentPose, drivetrain.XVel(), drivetrain.YVel());
    }

    public void rapidFire() {
        intake.runRapid();
        arms.arm3_flickRAPID();
    }

    public void neutral() {
        arms.reset();
        intake.stop();
    }

    public boolean systemsReady() {
        return drivetrain.DTatTarget() && differential.atTarget && flywheel.atTargetVelocity();
    }

    public void holdPoint(Pose2D targetPoint, double maxPower){
        drivetrain.state = Drivetrain.State.HOLD_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, 0, 0);
    }

    public Pose2D getTargetGoal() {
        if (alliance.isRed()) {
            return redGoal;
        } else {
            return blueGoal;
        }
    }

    public void readMotifTag(){
        if (limelight.getMotif() != 0) {
            MotifTagID = limelight.getMotif();
        }
    }

    // =========================
    // DEBUG TELEMETRY
    // =========================
    public double getOffsetX() { return odomOffset.getX(DistanceUnit.INCH); }
    public double getOffsetY() { return odomOffset.getY(DistanceUnit.INCH); }
    public boolean isLimelightActive() { return limelight.tagDetected; }

}
