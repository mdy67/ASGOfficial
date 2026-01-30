package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public Wait wait = new Wait();

    /* =========================
       FIELD CONSTANTS
       ========================= */
    public Pose2D blueGoal = new Pose2D(DistanceUnit.INCH, -64, 64, AngleUnit.DEGREES, 0);
    public Pose2D redGoal  = new Pose2D(DistanceUnit.INCH,  68, 64, AngleUnit.DEGREES, 0);

    public Pose2D adjustedPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public Pose2D finalAutoPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    /* =========================
       AUTO / STATE VARIABLES
       ========================= */
    public double MotifTagID = 21;
    public int splineCounter = 0;

    // legacy counter used by auto logic
    public int counter = 0;

    private Pose2D odomOffset = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private boolean offsetInitialized = false;

    private boolean AUTO_TESTING_MODE = false;


    /* =========================
       SHOOT 133 STATE MACHINE
       ========================= */
    private enum ShootState {
        IDLE,
        GO_SLOT_1,
        GO_SLOT_2,
        GO_SLOT_3,
        FLICK_AND_INTAKE,
        FLICK_AND_INTAKE_2,
        INTAKE,
        RAPID_FIRE,
        DONE
    }

    private ShootState shootState = ShootState.IDLE;
    private final ElapsedTime shootTimer = new ElapsedTime();

    /* =========================
       CONSTRUCTOR
       ========================= */
    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        drivetrain   = new Drivetrain(hardwareMap);
        differential = new Differential(hardwareMap);
        colors       = new Colors(hardwareMap);
        arms         = new Arms(hardwareMap);
        intake       = new Intake(hardwareMap);
        limelight    = new Limelight(hardwareMap);

        try {
            flywheel = new Flywheel(
                    hardwareMap,
                    hardwareMap.voltageSensor.iterator().hasNext()
                            ? hardwareMap.voltageSensor.iterator().next()
                            : null
            );
        } catch (Exception e) {
            flywheel = null;
        }
    }

    /* =========================
       STARTUP
       ========================= */
    public void startup() {
        arms.arm3_flickOFF();
        arms.arm2_flickOFF();
        arms.arm1_flickOFF();
        drivetrain.pinpoint.resetPosAndIMU();
        differential.resetEncoders();
     //   differential.encROffset = 0;
     //   differential.encLOffset = 0;
     //   differential.resetToSlot3();
    }

    public void importAutoPose(double x, double y, double heading) {
        drivetrain.setPosition(x, y, heading);
    }

    public void importAutoDiffy(double encLOffset, double encROffset) {
        differential.encLOffset = encLOffset;
        differential.encROffset = encROffset;
    }

    /* =========================
       UPDATE LOOP
       ========================= */
    public void update() {
        drivetrain.update();
        wait.update();
        flywheel.update();
        differential.update();
        limelight.update();

        if (AUTO_TESTING_MODE) {
            colors.updateTESTING(
                    drivetrain.DTatTarget(),
                    flywheel.atTargetVelocity(),
                    differential.atTarget
            );
        } else {
            colors.update();
        }

        Pose2D odomPose = drivetrain.robotPose;

        boolean hasValidTag =
                limelight.LimelightPose != null &&
                        limelight.LimelightPose.position.x != 0.0 &&
                        limelight.LimelightPose.position.y != 0.0 &&
                        limelight.tagDetected;

        boolean canFuse =
                hasValidTag &&
                        drivetrain.XVel() < 1.0 &&
                        drivetrain.YVel() < 1.0 &&
                        drivetrain.TVel() < 2.0;

        if (canFuse) {
            if (!offsetInitialized) {
                odomOffset = new Pose2D(
                        DistanceUnit.INCH,
                        limelight.LimelightPose.position.x - odomPose.getX(DistanceUnit.INCH),
                        limelight.LimelightPose.position.y - odomPose.getY(DistanceUnit.INCH),
                        AngleUnit.DEGREES,
                        0
                );
                offsetInitialized = true;
            } else {
                double newOffsetX =
                        odomOffset.getX(DistanceUnit.INCH) * 0.9 +
                                (limelight.LimelightPose.position.x - odomPose.getX(DistanceUnit.INCH)) * 0.1;

                double newOffsetY =
                        odomOffset.getY(DistanceUnit.INCH) * 0.9 +
                                (limelight.LimelightPose.position.y - odomPose.getY(DistanceUnit.INCH)) * 0.1;

                odomOffset = new Pose2D(
                        DistanceUnit.INCH,
                        newOffsetX,
                        newOffsetY,
                        AngleUnit.DEGREES,
                        0
                );
            }
        }

        adjustedPose = new Pose2D(
                DistanceUnit.INCH,
                odomPose.getX(DistanceUnit.INCH) + odomOffset.getX(DistanceUnit.INCH) + Limelight.X_OFFSET,
                odomPose.getY(DistanceUnit.INCH) + odomOffset.getY(DistanceUnit.INCH) + Limelight.Y_OFFSET,
                AngleUnit.DEGREES,
                odomPose.getHeading(AngleUnit.DEGREES)
        );
    }

    /* =========================
       DRIVETRAIN HELPERS
       ========================= */
    public void goToPoint(Pose2D targetPoint, double maxPower, double xyThreshold, double hThreshold) {
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, xyThreshold, hThreshold);
    }

    public void holdPoint(Pose2D targetPoint, double maxPower) {
        drivetrain.state = Drivetrain.State.HOLD_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, 0, 0);
    }

    /* =========================
       GOAL / SHOOTING HELPERS
       ========================= */
    public void goalLock(Pose2D currentPose) {
        differential.aimToGoal(currentPose, getTargetGoal());
        flywheel.aimToGoal(getTargetGoal(), currentPose, drivetrain.XVel(), drivetrain.YVel());
    }

    public void autoIdle() {
        if (alliance.isBlue()) {
            differential.setTargetAngle(150);
        } else {
            differential.setTargetAngle(30);
        }

        flywheel.setTargetVelocity(350);
        intake.stop();
        arms.reset();
    }

    public void rapidFire() {
        intake.runRapid();
        arms.arm3_flickRAPID();
    }

    public void rapidFire2(double power) {
        intake.runIntake(power);
        arms.arm3_flickRAPID();
    }

    public void neutral() {
        arms.reset();
        intake.stop();
      //  flywheel.stop();
    }

    public boolean systemsReady() {
        return drivetrain.DTatTarget()
                && differential.atTarget
                && flywheel.atTargetVelocity();
    }

    /* =========================
       SHOOT 133 (NON-BLOCKING)
       CALL EVERY LOOP
       ========================= */
    public boolean shoot_133(boolean goYet) {

        switch (shootState) {

            case IDLE:
                if (goYet) {
                    differential.goToSlot(1);
                    shootState = ShootState.GO_SLOT_1;
                    intake.runIntake(-0.8);
                    shootTimer.reset();
                }
                break;

            case GO_SLOT_1:
                if (differential.atTarget) {
                    if (shootTimer.seconds() >= 0.1) {
                        shootTimer.reset();
                        arms.arm1_flickON();
                        shootState = ShootState.FLICK_AND_INTAKE;
                    }


                }
                break;

            case FLICK_AND_INTAKE:
                if (shootTimer.seconds() >= 0.5) {
                    intake.stop();
                    arms.arm1_flickOFF();
                    differential.goToSlot(3);
                    shootState = ShootState.GO_SLOT_3;
                }
                break;

            case GO_SLOT_3:
                if (differential.atTarget) {
                    shootTimer.reset();
                    rapidFire2(-0.8);
                    shootState = ShootState.RAPID_FIRE;
                }
                break;

            case RAPID_FIRE:
                if (shootTimer.seconds() >= 0.6) {
                    neutral();
                    shootState = ShootState.DONE;
                }
                break;

            case DONE:
                shootState = ShootState.IDLE;
                return true;
        }

        return false;
    }

    public boolean shoot_223(boolean goYet) {

        switch (shootState) {

            case IDLE:
                if (goYet) {
                    differential.goToSlot(2);
                    shootState = ShootState.GO_SLOT_2;
                    intake.runIntake(-1.0);
                }
                break;

            case GO_SLOT_2:
                if (differential.atTarget) {
                    shootTimer.reset();
                    arms.arm2_flickON();
                    intake.runIntake(0.0);
                    shootState = ShootState.FLICK_AND_INTAKE;
                }
                break;

            case FLICK_AND_INTAKE:
                if (shootTimer.seconds() >= 0.3 && shootTimer.seconds() < 1.0) {
                    // intake.stop();
                    arms.arm2_flickOFF();
                    if (shootTimer.seconds() >= 0.6) {
                        intake.runIntake(-0.8);
                    }


                }
                if (shootTimer.seconds() >= 1.4 && shootTimer.seconds() < 1.8) {
                    intake.runIntake(0.0);
                    arms.arm2_flickON();
                }
                if (shootTimer.seconds() >= 1.8) {
                    intake.runIntake(-0.8);
                    arms.arm2_flickOFF();
                    shootTimer.reset();
                    differential.goToSlot(3);
                    shootState = ShootState.GO_SLOT_3;
                }
                break;

            case GO_SLOT_3:

                if (differential.atTarget) {
                    shootTimer.reset();
                    intake.runIntake(-1.0);
                    arms.arm3_flickON();
                    shootState = ShootState.RAPID_FIRE;
                }
                break;

            case RAPID_FIRE:
                if (shootTimer.seconds() >= 0.3) {
                    neutral();
                    shootState = ShootState.DONE;
                }
                break;

            case DONE:
                shootState = ShootState.IDLE;
                return true;
        }

        return false;
    }

    public boolean shoot_233(boolean goYet) {

        switch (shootState) {

            case IDLE:
                if (goYet) {
                    differential.goToSlot(2);
                    arms.reset();
                    shootState = ShootState.GO_SLOT_2;
                }
                break;

            case GO_SLOT_2:
                if (differential.atTarget) {
                    shootTimer.reset();
                    intake.runIntake(0);
                    arms.arm2_flickON();
                    shootState = ShootState.FLICK_AND_INTAKE;
                }
                break;

            case FLICK_AND_INTAKE:
                if (shootTimer.seconds() >= 0.3) {
                    intake.stop();
                    arms.arm2_flickOFF();
                    differential.goToSlot(3);
                    shootState = ShootState.GO_SLOT_3;
                }
                break;

            case GO_SLOT_3:
                if (differential.atTarget) {
                    shootTimer.reset();
                    rapidFire2(-0.8);
                    shootState = ShootState.RAPID_FIRE;
                }
                break;

            case RAPID_FIRE:
                if (shootTimer.seconds() >= 0.85) {
                    neutral();
                    shootState = ShootState.DONE;
                }
                break;

            case DONE:
                shootState = ShootState.IDLE;
                return true;
        }

        return false;
    }

    public boolean shoot_333(boolean goYet) {

        switch (shootState) {

            case IDLE:
                if (goYet) {
                    arms.reset();
                    differential.goToSlot(3);
                    shootState = ShootState.GO_SLOT_3;
                }
                break;

            case GO_SLOT_3:
                if (differential.atTarget) {
                    shootTimer.reset();
                    shootState = ShootState.RAPID_FIRE;
                }
                break;

            case RAPID_FIRE:
                rapidFire2(-0.8);
                if (shootTimer.seconds() >= 0.85) {
                    neutral();
                    shootState = ShootState.DONE;
                }
                break;

            case DONE:
                shootState = ShootState.IDLE;
                return true;
        }

        return false;
    }





    /* =========================
       AUTO UTILITIES
       ========================= */
    public Pose2D getTargetGoal() {
        return alliance.isRed() ? redGoal : blueGoal;
    }

    public void readMotifTag() {
        if (limelight.getMotif() != 0) {
            MotifTagID = limelight.getMotif();
        }
    }

    public void nextSplinePoint() {
        update();
        if (drivetrain.DTatTarget()) splineCounter++;
    }


    public void resetSplineCounter() {
        splineCounter = 0;
    }

    public double getOffsetX() { return odomOffset.getX(DistanceUnit.INCH); }
    public double getOffsetY() { return odomOffset.getY(DistanceUnit.INCH); }
    public boolean isLimelightActive() { return limelight.tagDetected; }
}
