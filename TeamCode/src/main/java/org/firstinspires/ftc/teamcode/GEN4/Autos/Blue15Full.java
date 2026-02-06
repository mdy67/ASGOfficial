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

@Autonomous(name = "Blue 15 Ball", group = "GEN4")
public class Blue15Full extends LinearOpMode {

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
            robot.differential.setTargetAngle(180);
            updateSequence();
        }

        // Setup motif-based patterns
        shootingPattern = new String[] {"133", "133", "133"};
        if (robot.MotifTagID == 21) {
            shootingPattern = new String[] {"133","233","333"};
        } else if (robot.MotifTagID == 22) {
            shootingPattern = new String[] {"223","333","233"};
        } else if (robot.MotifTagID == 23) {
            shootingPattern = new String[] {"333","133","223"};
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
                robot.drivetrain.setPosition(-16, -66, 90);
                robot.neutral();
                robot.readMotifTag();
                robot.flywheel.stop();
                telemetry.addData("INITIALIZED -- MOTIF:", robot.MotifTagID);
                telemetry.update();
                dtStallTimer.reset();
                break;

            case SHOOTING_POSE_1:

                if (dtStallTimer.seconds() > 1) {
                    robot.differential.aimToGoal(robot.drivetrain.robotPose, robot.getTargetGoal());
                    robot.flywheel.aimToGoal(robot.getTargetGoal(), robot.drivetrain.robotPose, robot.drivetrain.XVel(), robot.drivetrain.YVel());
                } else {
                    robot.flywheel.aimToGoal(robot.getTargetGoal(), robot.drivetrain.robotPose, robot.drivetrain.XVel(), robot.drivetrain.YVel());
                }

                robot.differential.farZone = true;
                if (!trigger) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, -57, AngleUnit.DEGREES, 240), 1, 3, 0.2);
                }

                if (robot.systemsReady() && !shootStarted) {
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
                        robot.flywheel.velocityModifier = -53.0; // VELOCITY OFFSETSET TODO: TUNE AT COMP
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
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -30, -53, AngleUnit.DEGREES, 210), 1, 6, 30);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    dtStallTimer.reset(); // RESET DT STALL TIMER
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -60, -50, AngleUnit.DEGREES, 250), 1, 4, 8);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 2) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -61, -58, AngleUnit.DEGREES, 255), 0.7, 3, 8);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 3) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -61, -65, AngleUnit.DEGREES, 270), 0.7, 5, 3);
                    checkStallTimer(2);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 4) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -28, -55, AngleUnit.DEGREES, 260), 1, 5, 20);
                    robot.nextSplinePoint();
                    if (robot.splineCounter == 5) robot.intake.runIntake(0);
                    trigger = false;
                } else if (robot.splineCounter == 5) {
                    robot.goalLock(robot.drivetrain.robotPose);

                    if (!trigger) {
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -10, -8, AngleUnit.DEGREES, 260), 1, 3, 0.2);
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
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -24, 10, AngleUnit.DEGREES, 180), 1, 4, Math.toRadians(20));
                    robot.nextSplinePoint();

                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -53, 10, AngleUnit.DEGREES, 180), 1, 2, 10);
                    robot.nextSplinePoint();
                    dtStallTimer.reset();

                } else if (robot.splineCounter == 2) {
                    robot.intake.runIntake(-1.0); // GATE INTERMEDIARY POS
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -46, 6, AngleUnit.DEGREES, 180), 1, 2, 10);
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
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -55, -2, AngleUnit.DEGREES, 180), 1, 2, 3);
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
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -10, 12, AngleUnit.DEGREES, 260), 1, 3, 0.2);
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
                            state = State.POINT_4;
                        }
                    }

                    robot.update();
                    if (robot.drivetrain.DTatTarget()) {
                        trigger = true;
                        robot.drivetrain.state = Drivetrain.State.IDLE;
                    }
                }
                break;

            case POINT_4:
                if (robot.splineCounter == 0) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -19, -15, AngleUnit.DEGREES, 180), 1, 5, 0.3);
                    robot.nextSplinePoint();

                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -59, -15, AngleUnit.DEGREES, 180), 1, 4, 10);
                    robot.goalLock(robot.drivetrain.robotPose);
                    robot.nextSplinePoint();
                    if (robot.splineCounter == 2) robot.intake.runIntake(0);
                    trigger = false;

                } else if (robot.splineCounter == 2) {
                    robot.goalLock(robot.drivetrain.robotPose);

                    if (!trigger) {
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -10, -8, AngleUnit.DEGREES, 260), 1, 3, 0.2);
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
                            state = State.POINT_5;
                        }
                    }

                    robot.update();
                    if (robot.drivetrain.DTatTarget()) {
                        trigger = true;
                        robot.drivetrain.state = Drivetrain.State.IDLE;
                    }
                }
                break;

            case POINT_5: // SHOOT "BOTTOM" 3 balls
                if (robot.splineCounter == 0) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -24, -38, AngleUnit.DEGREES, 180), 1, 3, 0.3);
                    robot.nextSplinePoint();
                    dtStallTimer.reset();

                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    checkStallTimer(2);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -64, -38, AngleUnit.DEGREES, 180), 1, 4, 10);
                    robot.nextSplinePoint();

                } else if (robot.splineCounter == 2) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -28, -15, AngleUnit.DEGREES, 260), 1, 15, 80);
                    robot.nextSplinePoint();
                    robot.goalLock(robot.drivetrain.robotPose);
                    if (robot.splineCounter == 3) robot.intake.runIntake(0);
                    trigger = false;
                } else if (robot.splineCounter == 3) {
                    robot.goalLock(robot.drivetrain.robotPose);

                    if (!trigger) {
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -10, -8, AngleUnit.DEGREES, 260), 1, 3, 0.2);
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
                            state = State.POINT_6;
                        }
                    }

                    robot.update();
                    if (robot.drivetrain.DTatTarget()) {
                        trigger = true;
                        robot.drivetrain.state = Drivetrain.State.IDLE;
                    }
                }
                break;

            case POINT_6: // MOVE OFF LINE
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -24, -10, AngleUnit.DEGREES, 270), 1, 3, 0.15);
                    robot.differential.aimToGoal(robot.drivetrain.robotPose, robot.getTargetGoal());
                    AutoToTeleop.storedPose = robot.drivetrain.robotPose;
                    alliance.set(alliance.Color.BLUE);
                    robot.update();
                    if (robot.drivetrain.DTatTarget()) {
                        robot.drivetrain.state = Drivetrain.State.IDLE;
                        state = State.FINISHED;
                    }
                break;
            case POINT_7:
            case POINT_8:
            case POINT_9:
            case FINISHED:
                AutoToTeleop.storedPose = robot.drivetrain.robotPose;
                alliance.set(alliance.Color.BLUE);
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
