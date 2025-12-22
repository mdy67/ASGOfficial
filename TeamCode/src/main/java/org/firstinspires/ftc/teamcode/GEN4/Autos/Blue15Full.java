package org.firstinspires.ftc.teamcode.GEN4.Autos;

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

        // INIT LOOP
        while (opModeInInit()) {
            state = State.INITIALIZED;
            updateSequence();
            telemetry.addData("AUTO FSM", state);
            telemetry.addData("DRIVETRAIN FSM", robot.drivetrain.state);
            telemetry.addData("Robot X", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("Robot Y", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("Robot Heading", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
            telemetry.update();
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

                if (!trigger) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, -55, AngleUnit.DEGREES, 240), 1, 1, 0.1);
                }

                if (robot.systemsReady() && !rapidFireActive) {
                    rapidFireTimer.reset();
                    rapidFireDuration = 1000; // 2 seconds
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
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -30, -53, AngleUnit.DEGREES, 210), 1, 4, 30);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -59, -50, AngleUnit.DEGREES, 240), 1, 4, 8);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 2) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -60, -58, AngleUnit.DEGREES, 245), 1, 3, 8);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 3) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -60, -66, AngleUnit.DEGREES, 270), 1, 3, 3);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 4) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -28, -55, AngleUnit.DEGREES, 260), 1, 4, 20);
                    robot.nextSplinePoint();
                    if (robot.splineCounter == 5) { robot.intake.runIntake(0); }
                    trigger = false;
                } else if (robot.splineCounter == 5) {
                    robot.goalLock(robot.drivetrain.robotPose);

                    if (!trigger) {
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 10, AngleUnit.DEGREES, 260), 1, 4, 0.1);
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
            // POINT_3
            // ----------------------------
            case POINT_3:
                if (robot.splineCounter == 0) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -24, -10, AngleUnit.DEGREES, 180), 1, 2, 2);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -55, -14, AngleUnit.DEGREES, 180), 1, 2, 10);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 2) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -48, -12, AngleUnit.DEGREES, 180), 1, 2, 10);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 3) {
                    robot.intake.runIntake(0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -52, -6, AngleUnit.DEGREES, 180), 1, 2, 3);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 4) {
                    robot.intake.runIntake(0);
                    robot.goalLock(robot.drivetrain.robotPose);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -20, -4, AngleUnit.DEGREES, 180), 1, 4, 10);
                    robot.nextSplinePoint();
                    trigger = false;
                } else if (robot.splineCounter == 5) {
                    robot.goalLock(robot.drivetrain.robotPose);

                    if (!trigger) {
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 10, AngleUnit.DEGREES, 260), 1, 4, 0.1);
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

            // ----------------------------
            // POINT_4
            // ----------------------------
            case POINT_4:
                if (robot.splineCounter == 0) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -24, -38, AngleUnit.DEGREES, 180), 1, 5, 10);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -58, -38, AngleUnit.DEGREES, 180), 1, 4, 10);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 2) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -28, -15, AngleUnit.DEGREES, 260), 1, 15, 80);
                    robot.nextSplinePoint();
                    robot.goalLock(robot.drivetrain.robotPose);
                    if (robot.splineCounter == 3) { robot.intake.runIntake(0); }
                    trigger = false;
                } else if (robot.splineCounter == 3) {
                    robot.goalLock(robot.drivetrain.robotPose);

                    if (!trigger) {
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 10, AngleUnit.DEGREES, 260), 1, 4, 0.1);
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
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -19, 12, AngleUnit.DEGREES, 180), 1, 2, 5);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -50, 12, AngleUnit.DEGREES, 180), 1, 2, 10);
                    robot.goalLock(robot.drivetrain.robotPose);
                    robot.nextSplinePoint();
                    trigger = false;
                } else if (robot.splineCounter == 2) {
                    robot.goalLock(robot.drivetrain.robotPose);

                    if (!trigger) {
                        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 10, AngleUnit.DEGREES, 260), 1, 4, 0.1);
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
