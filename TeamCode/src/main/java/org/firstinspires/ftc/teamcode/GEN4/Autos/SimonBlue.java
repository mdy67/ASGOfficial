package org.firstinspires.ftc.teamcode.GEN4.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.AutoToTeleop;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.alliance;

@Autonomous(name = "SIMON BLUE 9", group = "GEN4")
public class SimonBlue extends LinearOpMode {

    private Robot robot;

    // Pose storage for teleop

    public enum State {
        INITIALIZED,
        START_POSE,
        SHOOTING_POSE_1,
        FIRST_INTAKES,
        POINT_3,
        POINT_4,
        POINT_5,
        POINT_6,
        POINT_7,
        POINT_8,
        POINT_9,
        FINISHED
    }

    private State state = State.START_POSE;

    private ElapsedTime rapidFireTimer = new ElapsedTime();
    private boolean rapidFireActive = false;
    private long rapidFireDuration = 0;

    private boolean shootStarted = false;
    private String[] shootingPattern;

    private ElapsedTime dtStallTimer = new ElapsedTime();

    boolean trigger = false;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
        robot.startup();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        robot.flywheel.MAX_VELOCITY = 800;

        alliance.set(alliance.Color.BLUE);
        // INIT LOOP

        while (opModeInInit()) {
            state = State.INITIALIZED;
            robot.flywheel.stop();
            robot.differential.setTargetAngle(90);
            updateSequence();
        }

        waitForStart();
        state = State.SHOOTING_POSE_1;

        while (opModeIsActive()) {
            alliance.set(alliance.Color.BLUE);
            updateSequence();

            telemetry.addData("AUTO FSM", state);
            telemetry.addData("DRIVETRAIN FSM", robot.drivetrain.state);
            telemetry.addData("Robot X", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("Robot Y", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("Robot Heading", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("RapidFire Active", rapidFireActive);
            telemetry.addData("Shoot Started", shootStarted);

            telemetry.addData("Flywheel Target rad/s", robot.flywheel.getTargetVelocity());
            telemetry.addData("Flywheel Actual rad/s", robot.flywheel.getVelocityRadPerSec());

            telemetry.update();
            //     robot.finalAutoPose = robot.drivetrain.robotPose;
            AutoToTeleop.storedPose = robot.drivetrain.robotPose;
            alliance.set(alliance.Color.BLUE);

            if (robot.drivetrain.robotPose.getY(DistanceUnit.INCH) > -4) {
                robot.differential.SPECIAL_OFFSET = robot.drivetrain.robotPose.getY(DistanceUnit.INCH) * (3.50/60.0); // 2/60 [2 DEGREES AT 60 INCHES]
            } else {
                robot.differential.SPECIAL_OFFSET = 0;
            }

        }


        // Store the final pose for teleop
        //    robot.finalAutoPose = robot.drivetrain.robotPose;

        AutoToTeleop.storedPose = robot.drivetrain.robotPose;
        AutoToTeleop.encLOffset = robot.differential.currL;
        AutoToTeleop.encROffset = robot.differential.currR;
        //    robot.differential.encLOffset = robot.differential.currL;
        //    robot.differential.encROffset = robot.differential.currR;
        robot.update();
    }
    private void updateSequence() {
        switch (state) {

            case INITIALIZED:
                robot.update();
                robot.drivetrain.setPosition(-33, 63, 0);
                robot.neutral();
                robot.readMotifTag();
                robot.flywheel.stop();
                telemetry.addData("INITIALIZED -- MOTIF:", robot.MotifTagID);
                telemetry.update();
                dtStallTimer.reset();
                robot.differential.farZone = false;
                break;

            case SHOOTING_POSE_1:

                if (dtStallTimer.seconds() > 1) {
                    robot.differential.aimToGoal(robot.drivetrain.robotPose, robot.getTargetGoal());
                    robot.flywheel.aimToGoal(robot.getTargetGoal(), robot.drivetrain.robotPose, robot.drivetrain.XVel(), robot.drivetrain.YVel());
                } else {
                    robot.flywheel.aimToGoal(robot.getTargetGoal(), robot.drivetrain.robotPose, robot.drivetrain.XVel(), robot.drivetrain.YVel());
                }

                if (!trigger) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -10, 14, AngleUnit.DEGREES, 270), 1, 2, 0.2);
                }

                if (robot.systemsReady() && !shootStarted && dtStallTimer.seconds() > 4) {
                    shootStarted = true;
                    rapidFireTimer.reset();
                    rapidFireDuration = 1000;
                    rapidFireActive = true;
                }
                robot.flywheel.velocityModifier = 0.0; // VELOCITY OFFSETSET TODO: TUNE AT COMP
                if (rapidFireActive) {
                    robot.goalLock(robot.drivetrain.robotPose);
                    robot.rapidFire();

                    if (rapidFireTimer.milliseconds() >= rapidFireDuration) {
                        rapidFireActive = false;
                        shootStarted = false;
                        robot.autoIdle();
                        robot.resetSplineCounter();
                        robot.update();
                        state = State.FIRST_INTAKES;
                        robot.differential.farZone = false;
                        trigger = false;
                    }
                }

                robot.update();
                if (robot.drivetrain.DTatTarget()) {
                    trigger = true;
                    robot.drivetrain.state = Drivetrain.State.IDLE;

                }
                break;

            case FIRST_INTAKES: // INTAKE BALLS FROM CORNER
                if (robot.splineCounter == 0) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, -12, AngleUnit.DEGREES, 180), 1, 3, 0.2);
                    robot.nextSplinePoint();
                    dtStallTimer.reset(); // RESET DT STALL TIMER
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);

                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -60, -12, AngleUnit.DEGREES, 180), 1, 4, 1);
                    robot.nextSplinePoint();
                    checkStallTimer(2);
                } else if (robot.splineCounter == 2) {
                    dtStallTimer.reset(); // RESET DT STALL TIMER
                    robot.intake.runIntake(0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -40, -12, AngleUnit.DEGREES, 180), 1, 2, 1);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 3) { // GATE RAM POS
                    if (trigger) {
                        dtStallTimer.reset();
                        trigger = false;
                    }
                    // GATE RAM POS
                    robot.intake.runIntake(0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -53, -2, AngleUnit.DEGREES, 180), 1, 4, 3);
                    // WAIT 1s at GATE RAM POS
                    robot.update();
                    if (robot.drivetrain.DTatTarget() && !robot.wait.isFinished() && !robot.wait.isActive()) {
                        robot.wait.waitSeconds(1);
                    } else if (robot.wait.isFinished()) {
                        robot.nextSplinePoint();
                    }

                    checkStallTimer(2);

                    trigger = false;
                } else if (robot.splineCounter == 4) {

                    robot.intake.runIntake(0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -30, -6, AngleUnit.DEGREES, 260), 1, 4, 3);

                    robot.update();
                    robot.nextSplinePoint();

                    trigger = false;
                }

                else if (robot.splineCounter == 5) {
                    robot.goalLock(robot.drivetrain.robotPose);

                    if (!trigger) {
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -10, 14, AngleUnit.DEGREES, 260), 1, 3, 0.2);
                        dtStallTimer.reset();
                    }

                    if (robot.systemsReady() && !shootStarted && dtStallTimer.seconds() > 0.5) {
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
                            robot.update();
                            trigger = false;
                            state = State.POINT_3;
                        }
                    }

                    robot.update();
                    if (robot.drivetrain.DTatTarget()) {
                        trigger = true;
                        robot.drivetrain.state = Drivetrain.State.IDLE;
                    }
                }
                break;

            case POINT_3:
                if (robot.splineCounter == 0) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -24, 11, AngleUnit.DEGREES, 180), 1, 4, Math.toRadians(10));
                    robot.nextSplinePoint();

                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -53, 11, AngleUnit.DEGREES, 180), 1, 2, 10);
                    robot.nextSplinePoint();
                    dtStallTimer.reset();

                } else if (robot.splineCounter == 2) {
                    robot.intake.runIntake(-1.0); // GATE INTERMEDIARY POS
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -40, 7, AngleUnit.DEGREES, 180), 1, 2, 10);
                    robot.nextSplinePoint();
                    checkStallTimer(2);
                    trigger = true;
                } else if (robot.splineCounter == 3) {
                    if (trigger) {
                        dtStallTimer.reset();
                        trigger = false;
                    }
                    robot.goalLock(robot.drivetrain.robotPose);
                    robot.intake.runIntake(0); // GATE RAM POS
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -53, -4, AngleUnit.DEGREES, 180), 1, 2, 3);
                    // WAIT 1s at GATE RAM POS
                    robot.update();
                    if (robot.drivetrain.DTatTarget() && !robot.wait.isFinished() && !robot.wait.isActive()) {
                        robot.wait.waitSeconds(1);
                    } else if (robot.wait.isFinished()) {
                        robot.nextSplinePoint();
                    }

                    checkStallTimer(2);

                    trigger = false;
                    robot.counter = 0;


                } else if (robot.splineCounter == 4) {
                    robot.goalLock(robot.drivetrain.robotPose);

                    if (!trigger) {
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -12, 27, AngleUnit.DEGREES, 330), 1, 3, 0.2);
                        dtStallTimer.reset();
                    }

                    if (robot.systemsReady() && !shootStarted && dtStallTimer.seconds() > 0.5) {
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
                            robot.update();
                            trigger = false;
                            state = State.FINISHED;
                        }
                    }

                    robot.update();
                    if (robot.drivetrain.DTatTarget()) {
                        trigger = true;
                        robot.drivetrain.state = Drivetrain.State.IDLE;
                    }
                }
                break;

            case POINT_7:
            case POINT_8:
            case POINT_9:
            case FINISHED:
                AutoToTeleop.storedPose = robot.drivetrain.robotPose;
                alliance.set(alliance.Color.BLUE);
                robot.update();
                AutoToTeleop.storedPose = robot.drivetrain.robotPose;
                alliance.set(alliance.Color.BLUE);
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
