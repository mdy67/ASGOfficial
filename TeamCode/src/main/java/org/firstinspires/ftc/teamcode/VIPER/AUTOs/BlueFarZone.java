package org.firstinspires.ftc.teamcode.VIPER.AUTOs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.GEN4.OLDsubsystems.AutoToTeleop;
import org.firstinspires.ftc.teamcode.VIPER.*;

@Autonomous(name= "Blue Far Zone", group = "GEN4")
public class BlueFarZone extends LinearOpMode {

    private Robot robot;

    private int goonCounter = 0;
    private int splineCounter = 0;

    private ElapsedTime settleTimer = new ElapsedTime();
    private ElapsedTime dtStallTimer = new ElapsedTime();
    private ElapsedTime rapidFireTimer = new ElapsedTime();
    private ElapsedTime gameTimer = new ElapsedTime();
    private ElapsedTime loopCountTimer = new ElapsedTime();

    private boolean atTarget = false;
    private boolean firing = false;

    private double kTime;
    private double minReduc;

    public enum State {
        SHOOTING_POSE_1,
        CORNER_INTAKES,
        CORNER_INTAKES_2,
        THIRD_SPIKE,
        OFF_LINE_POS
    }

    private State state = State.SHOOTING_POSE_1;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        kTime = robot.shooter.kTime;
        minReduc = robot.shooter.maxPower;
        alliance.set(alliance.Color.BLUE);
        robot.drivetrain.setPosition(-8, -62.5, 180); // TODO: TUNE AT WORLDS
        robot.update(1);
        double counter = 0;

        while (opModeInInit()) {
            robot.turret.ANGLE_ADJUST = 5.8; // TODO: CHAGNE IF NEEDED
            if (counter < 100) {
                robot.drivetrain.setPosition(-8, -62.5, 180); // TODO: TUNE AT WORLDS
            }

            telemetry.addLine("IN INIT");
            telemetry.addLine();
            telemetry.addData("ROBOT X:", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("ROBOT Y:", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("ROBOT HEADING:", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            robot.shooter.stop();
            telemetry.addData("COUNTER:", counter);
            robot.update(1);


            counter ++;



        }

        waitForStart();
        gameTimer.reset();


        while (opModeIsActive()) {
            telemetry.addData("LOOP TIME:", loopCountTimer.seconds());
            telemetry.update();
            loopCountTimer.reset();
            updateSequence();
            if (state != State.THIRD_SPIKE) {
                robot.update(1);
            } else {
                robot.update(3);
            }

            if (gameTimer.seconds() > 28.75) {
                state = State.OFF_LINE_POS;
                goonCounter = 7;
            }

            alliance.set(alliance.Color.BLUE);
            AutoToTeleop.storedPose = robot.drivetrain.robotPose;

        }
    }

    private void updateSequence() {

        switch (state) {

            // =========================
            // 🔥 SHOOTING
            // =========================
            case SHOOTING_POSE_1:

                if (!atTarget) {

                    robot.intake.setPower(-1);
                    robot.linkage.ON();
                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, -22, -56, AngleUnit.DEGREES, 210),
                            1, 3, 0.08
                    );

                    if (robot.drivetrain.DTatTarget() && robot.intake.current() >= 2.5) {
                        atTarget = true;
                        settleTimer.reset();
                        robot.drivetrain.state = Drivetrain.State.HOLD_POINT;
                    } else if (robot.drivetrain.DTatTarget() && goonCounter > 0) {
                        firing = false;
                        atTarget = false;

                        robot.shooter.shooting = false;
                      //  robot.intake.setPower(0);
                        robot.linkage.ON();

                        dtStallTimer.reset();
                        splineCounter = 0;
                        state = State.CORNER_INTAKES;
                    } else if (dtStallTimer.seconds() > 2) {
                        atTarget = true;
                        settleTimer.reset();
                        robot.drivetrain.state = Drivetrain.State.HOLD_POINT;
                    }
                }

                else {

                    // MUST STILL BE STABLE
                    if (settleTimer.seconds() > 0.2 && robot.drivetrain.DTatTarget() && robot.shooter.atVelocity()) {

                        robot.linkage.OFF();

                        if (!firing) {
                            rapidFireTimer.reset();
                            firing = true;
                        }

                        robot.shooter.shooting = true;

                        robot.intake.setPower(
                                Range.clip(-1 + (rapidFireTimer.seconds() * kTime), -1, minReduc)
                        );

                        // DONE FIRING
                        if (rapidFireTimer.seconds() > 0.9) {

                            firing = false;
                            atTarget = false;

                            robot.shooter.shooting = false;
                            robot.intake.setPower(0);
                            robot.linkage.ON();

                            dtStallTimer.reset();
                            splineCounter = 0;

                            if (goonCounter == 0) {
                                state = State.CORNER_INTAKES;
                            } else if (goonCounter == 1) {
                                state = State.THIRD_SPIKE;
                            } else if (goonCounter == 2) {
                                state = State.CORNER_INTAKES_2;
                            } else if (goonCounter == 3) {
                                state = State.CORNER_INTAKES;
                            } else if (goonCounter == 4) {
                                state = State.CORNER_INTAKES_2;
                            } else if (goonCounter == 5) {
                                state = State.CORNER_INTAKES_2;
                            } else if (goonCounter == 6) {
                                state = State.CORNER_INTAKES;
                            } else if (goonCounter == 7) {
                                state = State.OFF_LINE_POS;
                            }


                            goonCounter++;
                        }
                    }
                }
                break;

            // =========================
            // 🟦 CORNER INTAKE (FIXED)
            // =========================
            case CORNER_INTAKES:

                robot.intake.setPower(-1);
                robot.linkage.ON();

                if (splineCounter == 0) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, -66, -65, AngleUnit.DEGREES, 180),
                            1, 3, 0.15
                    );

                    if (robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.5 || dtStallTimer.seconds() > 1.5) {
                        splineCounter = 1;
                    }

                } else if (splineCounter == 1) {

                        splineCounter = 0;
                        dtStallTimer.reset();
                        state = State.SHOOTING_POSE_1;
                }

                break;

            // =========================
            // 🟦 CORNER INTAKE 2 (UNCHANGED)
            // =========================
            case CORNER_INTAKES_2:

                robot.intake.setPower(-1);
                robot.linkage.ON();

                if (splineCounter == 0) {

                    if (dtStallTimer.seconds() == 0) {
                        dtStallTimer.reset(); // ensure started once
                    }

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, -50, -35, AngleUnit.DEGREES, 180),
                            1, 5, 0.25
                    );

                    // 🔒 REQUIRE MIN TIME BEFORE ALLOWING TRANSITION
                    if ((robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2)
                            || dtStallTimer.seconds() > 1.0) {

                        splineCounter = 1;
                        dtStallTimer.reset();
                    }

                } else if (splineCounter == 1) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, -66, -35, AngleUnit.DEGREES, 180),
                            1, 3, 0.25
                    );

                    if (robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2 || dtStallTimer.seconds() > 0.5) {
                        splineCounter = 0;
                        dtStallTimer.reset();
                        state = State.SHOOTING_POSE_1;
                    }
                }

                break;

            case THIRD_SPIKE:

                robot.intake.setPower(-1);
                robot.linkage.ON();

                if (splineCounter == 0) {

                    if (dtStallTimer.seconds() == 0) {
                        dtStallTimer.reset(); // ensure started once
                    }

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, -48, -48, AngleUnit.DEGREES, 90),
                            1, 3, 0.08
                    );

                    // 🔒 REQUIRE MIN TIME BEFORE ALLOWING TRANSITION
                    if ((robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.8)
                            || dtStallTimer.seconds() > 1.0) {

                        splineCounter = 1;
                        dtStallTimer.reset();
                    }

                } else if (splineCounter == 1) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, -48, -30, AngleUnit.DEGREES, 90),
                            1, 3, 0.08
                    );

                    if (robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.5 || dtStallTimer.seconds() > 1.5) {
                        splineCounter = 0;
                        dtStallTimer.reset();
                        state = State.SHOOTING_POSE_1;
                    }
                }

                break;

            // =========================
            // 🟩 PARK
            // =========================
            case OFF_LINE_POS:

                robot.intake.setPower(0);

                robot.goToPoint(
                        new Pose2D(DistanceUnit.INCH, -40, -40, AngleUnit.DEGREES, 180),
                        1, 3, 0.25
                );

                break;
        }
    }
}