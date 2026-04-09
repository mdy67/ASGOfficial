package org.firstinspires.ftc.teamcode.VIPER.AUTOs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.VIPER.*;

@Autonomous(name= "Blue Far Zone", group = "GEN4")
public class BlueFarZone extends LinearOpMode {

    private Robot robot;

    private int goonCounter = 0;
    private int splineCounter = 0;

    private ElapsedTime settleTimer = new ElapsedTime();
    private ElapsedTime dtStallTimer = new ElapsedTime();
    private ElapsedTime rapidFireTimer = new ElapsedTime();

    private boolean atTarget = false;
    private boolean firing = false;

    private double kTime = 0.41;

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
        alliance.set(alliance.Color.BLUE);

        while (opModeInInit()) {
            robot.shooter.stop();
            robot.drivetrain.setPosition(-8, -63, 180);
            robot.update(1);
        }

        waitForStart();

        while (opModeIsActive()) {
            updateSequence();
            if (state != State.THIRD_SPIKE) {
                robot.update(1);
            } else {
                robot.update(4);
            }

        }
    }

    private void updateSequence() {

        switch (state) {

            // =========================
            // 🔥 SHOOTING
            // =========================
            case SHOOTING_POSE_1:

                if (!atTarget) {

                    robot.intake.setPower(-0.4);

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, -20, -57, AngleUnit.DEGREES, 180),
                            1, 3, 0.08
                    );

                    if (robot.drivetrain.DTatTarget()) {
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
                                Range.clip(-1 + (rapidFireTimer.seconds() * kTime), -1, -0.76)
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
                                state = State.CORNER_INTAKES;
                            } else if (goonCounter == 6) {
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
                            new Pose2D(DistanceUnit.INCH, -65, -64, AngleUnit.DEGREES, 180),
                            0.7, 3, 0.15
                    );

                    if (robot.drivetrain.DTatTarget() || dtStallTimer.seconds() > 1.0) {
                        splineCounter = 1;
                    }

                } else if (splineCounter == 1) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, -65, -64, AngleUnit.DEGREES, 180),
                            1, 3, 0.25
                    );

                    if (robot.drivetrain.DTatTarget() || dtStallTimer.seconds() > 1.0) {
                        splineCounter = 0;
                        state = State.SHOOTING_POSE_1;
                    }
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
                            new Pose2D(DistanceUnit.INCH, -40, -15, AngleUnit.DEGREES, 180),
                            1, 3, 0.25
                    );

                    // 🔒 REQUIRE MIN TIME BEFORE ALLOWING TRANSITION
                    if ((robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2)
                            || dtStallTimer.seconds() > 1.0) {

                        splineCounter = 1;
                        dtStallTimer.reset();
                    }

                } else if (splineCounter == 1) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, -66, -20, AngleUnit.DEGREES, 180),
                            1, 3, 0.25
                    );

                    if (robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2 || dtStallTimer.seconds() > 1) {
                        splineCounter = 0;
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
                            || dtStallTimer.seconds() > 4.0) {

                        splineCounter = 1;
                        dtStallTimer.reset();
                    }

                } else if (splineCounter == 1) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, -48, -30, AngleUnit.DEGREES, 90),
                            1, 3, 0.08
                    );

                    if (robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.5 || dtStallTimer.seconds() > 4.5) {
                        splineCounter = 0;
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