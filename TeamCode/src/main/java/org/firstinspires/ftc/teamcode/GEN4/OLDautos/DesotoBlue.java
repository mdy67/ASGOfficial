package org.firstinspires.ftc.teamcode.GEN4.OLDautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.OLDsubsystems.AutoToTeleop;
import org.firstinspires.ftc.teamcode.GEN4.OLDsubsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GEN4.OLDsubsystems.Robot;
import org.firstinspires.ftc.teamcode.GEN4.OLDsubsystems.alliance;

@Autonomous(name = "Desoto BLUECHEW", group = "GEN4")
public class DesotoBlue extends LinearOpMode {

    private Robot robot;

    public enum State {
        INITIALIZED,
        START_POSE,
        SHOOTING_POSE_1,
        SPIKE_INTAKES,
        SHOOTING_POSE_2,
        POOL_INTAKE,
        CORNER_INTAKE,
        OFFLINE,
        POINT_6,
        POINT_7,
        POINT_8,
        POINT_9,
        FINISHED
    }

    private State state = State.START_POSE;

    private ElapsedTime rapidFireTimer = new ElapsedTime();
    private ElapsedTime dtStallTimer = new ElapsedTime();
    private boolean rapidFireActive = false;
    private long rapidFireDuration = 0;

    private double CycleCounter = 0;

    private boolean shootStarted = false;
    boolean trigger = false;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
        robot.startup();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        robot.flywheel.MAX_VELOCITY = 850;
        alliance.set(alliance.Color.BLUE);

        while (opModeInInit()) {
            state = State.INITIALIZED;
            robot.flywheel.stop();
            updateSequence();
        }

        waitForStart();
        state = State.SHOOTING_POSE_1;

        while (opModeIsActive()) {
            alliance.set(alliance.Color.BLUE);
            updateSequence();

            telemetry.update();
            AutoToTeleop.storedPose = robot.drivetrain.robotPose;
        }

        AutoToTeleop.storedPose = robot.drivetrain.robotPose;
        robot.update();
    }

    private void updateSequence() {
        switch (state) {
            case INITIALIZED:
                // robot.differential.setTargetAngle(90);
                robot.update();
                robot.differential.setTargetAngle(0);
                robot.drivetrain.setPosition(-16, -66, 90);
                robot.neutral();
                robot.readMotifTag();
                robot.flywheel.stop();
                telemetry.addData("INITIALIZED -- MOTIF:", robot.MotifTagID);
                telemetry.update();
                dtStallTimer.reset();
                break;
            /* ================= SHOOT 1 ================= */

            case SHOOTING_POSE_1:

                if (dtStallTimer.seconds() < 1) {
                    robot.flywheel.aimToGoal(robot.getTargetGoal(), robot.drivetrain.robotPose, robot.drivetrain.XVel(), robot.drivetrain.YVel());
                } else {
                    robot.goalLock(robot.drivetrain.robotPose);
                }

                robot.differential.farZone = true;

                if (!trigger) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -17, -57, AngleUnit.DEGREES, 240), 1, 3, 0.15);
                }

                if (robot.systemsReady() && !shootStarted) {
                    shootStarted = true;
                    rapidFireTimer.reset();
                    rapidFireDuration = 1000;
                    rapidFireActive = true;
                }

                if (rapidFireActive) {
                    robot.goalLock(robot.drivetrain.robotPose);
                    robot.rapidFire();

                    if (rapidFireTimer.milliseconds() >= rapidFireDuration) {
                        rapidFireActive = false;
                        shootStarted = false;
                        robot.autoIdle();
                        robot.resetSplineCounter();
                        state = State.CORNER_INTAKE;
                        trigger = false;
                    }
                }

                robot.update();
                if (robot.drivetrain.DTatTarget()) {
                    trigger = true;
                    robot.drivetrain.state = Drivetrain.State.IDLE;
                }
                break;

            case CORNER_INTAKE:
                if (robot.splineCounter == 0) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -30, -53, AngleUnit.DEGREES, 210), 1, 6, 30);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -60, -50, AngleUnit.DEGREES, 240), 1, 4, 8);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 2) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -61, -58, AngleUnit.DEGREES, 255), 0.5, 3, 8);
                    robot.nextSplinePoint();
                    dtStallTimer.reset();
                } else if (robot.splineCounter == 3) {
                    checkStallTimer(1);
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -61, -66, AngleUnit.DEGREES, 270), 0.5, 5, 3);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 4) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -40, -60, AngleUnit.DEGREES, 270), 0.5, 5, 3);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 5) {
                    robot.intake.runIntake(0);
                    robot.resetSplineCounter();
                    trigger = false;
                    robot.update();
                    state = State.SHOOTING_POSE_2;
                }
                break;

            case SPIKE_INTAKES:
                if (robot.splineCounter == 0) {
                    dtStallTimer.reset();
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -24, -38, AngleUnit.DEGREES, 180), 1, 3, 0.15);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -61, -38, AngleUnit.DEGREES, 180), 1, 4, 0.15);
                    robot.nextSplinePoint();
                    checkStallTimer(2);
                } else if (robot.splineCounter == 2) {
                    robot.intake.runIntake(0);
                    robot.resetSplineCounter();
                    trigger = false;
                    robot.update();
                    state = State.SHOOTING_POSE_2;
                }
                break;

            case SHOOTING_POSE_2:
                robot.goalLock(robot.drivetrain.robotPose);
                robot.differential.farZone = true;

                if (!trigger) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -17, -57, AngleUnit.DEGREES, 240), 1, 2, 0.15);
                }

                if (robot.systemsReady() && !shootStarted && dtStallTimer.seconds() > 0.75) {
                    shootStarted = true;
                    rapidFireTimer.reset();
                    rapidFireDuration = 1000;
                    rapidFireActive = true;
                }

                if (rapidFireActive) {
                    robot.goalLock(robot.drivetrain.robotPose);
                    robot.rapidFire();

                    if (rapidFireTimer.milliseconds() >= rapidFireDuration) {
                        rapidFireActive = false;
                        shootStarted = false;
                        robot.autoIdle();
                        robot.resetSplineCounter();
                        dtStallTimer.reset();
                        CycleCounter ++;
                        if (CycleCounter == 1) {
                            state = State.SPIKE_INTAKES;
                        } else if (CycleCounter < 4){
                            state = State.POOL_INTAKE;
                        } else {
                            state = State.OFFLINE;
                        }
                        trigger = false;
                    }
                }

                robot.update();
                if (robot.drivetrain.DTatTarget()) {
                    if (!trigger) { dtStallTimer.reset(); }
                    trigger = true;



                    robot.drivetrain.state = Drivetrain.State.IDLE;
                }
                break;

            case POOL_INTAKE:
                if (robot.splineCounter == 0) {
                    //  checkStallTimer(3);
                    dtStallTimer.reset();
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -50, -62, AngleUnit.DEGREES, 180), 0.5, 3, 0.15);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -68, -68, AngleUnit.DEGREES, 180), 0.4, 2, 0.15);
                    robot.nextSplinePoint();
                    checkStallTimer(1.5);
                } else if (robot.splineCounter == 2) {
                    robot.intake.runIntake(0);
                    robot.resetSplineCounter();
                    trigger = false;
                    state = State.SHOOTING_POSE_2;
                }
                break;

            case OFFLINE: // MOVE OFF LINE
                if (robot.splineCounter == 0) {
                    checkStallTimer(3);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, -40, AngleUnit.DEGREES, 240), 0.3, 3, 0.15);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    state = State.FINISHED;
                }
                break;

            case FINISHED:
                robot.differential.aimToGoal(robot.drivetrain.robotPose, robot.getTargetGoal());
                robot.update();

                break;
        }
    }

    public void checkStallTimer(double threshold) {
        if (dtStallTimer.seconds() > threshold) {
            robot.splineCounter ++;
            dtStallTimer.reset();
        }
    }
}




