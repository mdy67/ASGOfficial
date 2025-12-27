package org.firstinspires.ftc.teamcode.GEN4.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;

@Autonomous(name = "Blue 15 Ball", group = "GEN4")
public class Blue15Full extends LinearOpMode {

    private Robot robot;

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

    // Timer for rapid fire
    private ElapsedTime rapidFireTimer = new ElapsedTime();
    private boolean rapidFireActive = false;
    private long rapidFireDuration = 0;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
        robot.startup();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // INIT LOOP
        while (opModeInInit()) {
            state = State.INITIALIZED;
            updateSequence();

        }

        waitForStart();
        state = State.SHOOTING_POSE_1;

        while (opModeIsActive() && !state.equals(State.FINISHED)) {
            updateSequence();

            telemetry.addData("AUTO FSM", state);
            telemetry.addData("DRIVETRAIN FSM", robot.drivetrain.state);
            telemetry.addData("Robot X", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("Robot Y", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("Robot Heading", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("RapidFire Active", rapidFireActive);

            telemetry.addData("Flywheel Target rad/s", robot.flywheel.getTargetVelocity());
            telemetry.addData("Flywheel Actual rad/s", robot.flywheel.getVelocityRadPerSec());

            telemetry.update();
        }
    }

    boolean trigger = false;
    private void updateSequence() {

        switch (state) {

            case INITIALIZED:
                robot.update();
                robot.drivetrain.setPosition(-16, -66, 90);
                robot.neutral();
                robot.readMotifTag();
                telemetry.addData("MOTIF:", robot.MotifTagID);
                telemetry.update();
                break;

            // ----------------------------
            // FIRST SHOOTING POINT
            // ----------------------------
            case SHOOTING_POSE_1:
                robot.goalLock(robot.drivetrain.robotPose);
                robot.differential.farZone = true;
                if (!trigger) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, -55, AngleUnit.DEGREES, 240), 1, 3, 0.2);
                }


                if (robot.systemsReady() && !rapidFireActive) {
                    rapidFireTimer.reset();
                    rapidFireDuration = 1000;
                    rapidFireActive = true;
                }

                if (rapidFireActive) {
                    robot.goalLock(robot.drivetrain.robotPose);
                    robot.rapidFire();

                    if (rapidFireTimer.milliseconds() >= rapidFireDuration) {
                        rapidFireActive = false;
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

            // ----------------------------
            // FIRST INTAKES + SHOOTING
            // ----------------------------
            case FIRST_INTAKES:

                if (robot.splineCounter == 0) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -30, -53, AngleUnit.DEGREES, 210), 1, 6, 30);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -59, -50, AngleUnit.DEGREES, 240), 1, 4, 8);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 2) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -60, -58, AngleUnit.DEGREES, 245), 0.25, 3, 8);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 3) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -60, -66, AngleUnit.DEGREES, 270), 0.25, 3, 3);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 4) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -28, -55, AngleUnit.DEGREES, 260), 1, 5, 20);
                    robot.nextSplinePoint();
                    if (robot.splineCounter == 5) robot.intake.runIntake(0);
                    trigger = false;
                } else if (robot.splineCounter == 5) {
                    robot.goalLock(robot.drivetrain.robotPose);

                    if (!trigger) {
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 1, AngleUnit.DEGREES, 260), 1, 3, 0.2);
                    }

                    if (robot.systemsReady() && !rapidFireActive) {
                        rapidFireTimer.reset();
                        rapidFireDuration = 1000;
                        rapidFireActive = true;
                    }

                    if (rapidFireActive) {
                        robot.goalLock(robot.drivetrain.robotPose);
                        robot.rapidFire();
                        if (rapidFireTimer.milliseconds() >= rapidFireDuration) {
                            rapidFireActive = false;
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

            // ----------------------------
            // POINT_3 [FIRST SORTING OCCURRANCE]
            // ----------------------------
            case POINT_3:
                if (robot.splineCounter == 0) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -24, 10, AngleUnit.DEGREES, 180), 1, 4, Math.toRadians(20));
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -48, 10, AngleUnit.DEGREES, 180), 1, 2, 10);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 2) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -44, 4, AngleUnit.DEGREES, 180), 1, 2, 10);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 3) {
                    robot.goalLock(robot.drivetrain.robotPose);
                    robot.intake.runIntake(0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -53, 1, AngleUnit.DEGREES, 180), 1, 2, 3);
                    robot.nextSplinePoint();
                    trigger = false;

                    robot.counter = 0;
                } else if (robot.splineCounter == 4) {


                    if (!trigger) {
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 1, AngleUnit.DEGREES, 260), 1, 3, 0.2);
                        robot.goalLock(robot.drivetrain.robotPose);
                    }

                    robot.shoot_133(90);
                    if (robot.counter == 3) {
                        rapidFireActive = false;
                        robot.resetSplineCounter();
                        robot.update();
                        trigger = false;
                        state = State.POINT_4;
                    }
                    /*
                    if (robot.systemsReady() && !rapidFireActive) {
                        rapidFireTimer.reset();
                        rapidFireDuration = 1000;
                        rapidFireActive = true;
                    }

                    if (rapidFireActive) {
                        robot.goalLock(robot.drivetrain.robotPose);
                        robot.rapidFire();
                        if (rapidFireTimer.milliseconds() >= rapidFireDuration) {
                            rapidFireActive = false;
                            robot.autoIdle();
                            robot.resetSplineCounter();
                            robot.update();
                            trigger = false;
                            state = State.POINT_4;
                        }
                    }
                    */

                    robot.update();
                    if (robot.drivetrain.DTatTarget()) {
                        trigger = true;
                        robot.drivetrain.state = Drivetrain.State.IDLE;
                    }
                }
                break;

            // ----------------------------
            // POINT_4
            // ----------------------------
            case POINT_4:
                if (robot.splineCounter == 0) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -19, -12, AngleUnit.DEGREES, 180), 1, 5, 0.3);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -53, -15, AngleUnit.DEGREES, 180), 1, 4, 10);
                    robot.goalLock(robot.drivetrain.robotPose);
                    robot.nextSplinePoint();
                    if (robot.splineCounter == 2) robot.intake.runIntake(0);
                    trigger = false;
                } else if (robot.splineCounter == 2) {
                    robot.goalLock(robot.drivetrain.robotPose);

                    if (!trigger) {
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 1, AngleUnit.DEGREES, 260), 1, 3, 0.2);
                    }

                    if (robot.systemsReady() && !rapidFireActive) {
                        rapidFireTimer.reset();
                        rapidFireDuration = 1000;
                        rapidFireActive = true;
                    }

                    if (rapidFireActive) {
                        robot.goalLock(robot.drivetrain.robotPose);
                        robot.rapidFire();
                        if (rapidFireTimer.milliseconds() >= rapidFireDuration) {
                            rapidFireActive = false;
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

            // ----------------------------
            // POINT_5
            // ----------------------------
            case POINT_5:
                if (robot.splineCounter == 0) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -24, -38, AngleUnit.DEGREES, 180), 1, 6, 10);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -58, -38, AngleUnit.DEGREES, 180), 1, 4, 10);
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
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -16, 12, AngleUnit.DEGREES, 310), 1, 3, 0.2);
                    }

                    if (robot.systemsReady() && !rapidFireActive) {
                        rapidFireTimer.reset();
                        rapidFireDuration = 1000;
                        rapidFireActive = true;
                    }

                    if (rapidFireActive) {
                        robot.goalLock(robot.drivetrain.robotPose);
                        robot.rapidFire();
                        if (rapidFireTimer.milliseconds() >= rapidFireDuration) {
                            rapidFireActive = false;
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

            case POINT_6:
            case POINT_7:
            case POINT_8:
            case POINT_9:
            case FINISHED:
                robot.update();
                break;
        }
    }
}
