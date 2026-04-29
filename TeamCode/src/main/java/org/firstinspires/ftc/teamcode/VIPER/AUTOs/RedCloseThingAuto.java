package org.firstinspires.ftc.teamcode.VIPER.AUTOs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.GEN4.OLDsubsystems.AutoToTeleop;
import org.firstinspires.ftc.teamcode.VIPER.*;

@Autonomous(name= "Red Close Auto", group = "GEN4")
public class RedCloseThingAuto extends LinearOpMode {

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
        TOP_SPIKE,
        MID_SPIKE,
        GATE_INTAKES,
        OFF_LINE_POS
    }

    private State state = State.SHOOTING_POSE_1;
    private double GATE_HIT_X_POS = 57;   // flipped
    private double SPIKE_X_POS = 48;      // flipped

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        kTime = robot.shooter.kTime;
        minReduc = robot.shooter.maxPower;
        alliance.set(alliance.Color.RED);

        double counter = 0;

        while (opModeInInit()) {
            if (counter < 200) {
                robot.drivetrain.setPosition(35, 64, 270); // mirrored
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

            robot.turret.ANGLE_ADJUST = -1; // TODO: CHAGNE IF NEEDED
            counter ++;
        }

        waitForStart();
        gameTimer.reset();

        while (opModeIsActive()) {
            telemetry.addData("LOOP TIME:", loopCountTimer.seconds());
            telemetry.update();
            loopCountTimer.reset();
            updateSequence();
            robot.update(1);

            if (gameTimer.seconds() > 28.75) {
                state = State.OFF_LINE_POS;
                goonCounter = 7;
            }

            alliance.set(alliance.Color.RED);
            AutoToTeleop.storedPose = robot.drivetrain.robotPose;
        }
    }

    private void updateSequence() {

        switch (state) {

            case SHOOTING_POSE_1:

                if (!atTarget) {

                    robot.intake.setPower(-1);
                    robot.linkage.ON();
                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, 22, 10, AngleUnit.DEGREES, 310),
                            1, 3, 0.08
                    );

                    if (robot.drivetrain.DTatTarget()) {
                        atTarget = true;
                        settleTimer.reset();
                        robot.drivetrain.state = Drivetrain.State.HOLD_POINT;
                    } else if (robot.drivetrain.DTatTarget() && goonCounter > 0) {
                        firing = false;
                        atTarget = false;

                        robot.shooter.shooting = false;
                        robot.linkage.ON();

                        dtStallTimer.reset();
                        splineCounter = 0;
                        state = State.TOP_SPIKE;
                    } else if (dtStallTimer.seconds() > 2) {
                        atTarget = true;
                        settleTimer.reset();
                        robot.drivetrain.state = Drivetrain.State.HOLD_POINT;
                    }
                }

                else {

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

                        if (rapidFireTimer.seconds() > 0.9) {

                            firing = false;
                            atTarget = false;

                            robot.shooter.shooting = false;
                            robot.intake.setPower(0);
                            robot.linkage.ON();

                            dtStallTimer.reset();
                            splineCounter = 0;

                            if (goonCounter == 0) {
                                state = State.TOP_SPIKE;
                            } else if (goonCounter == 1) {
                                state = State.MID_SPIKE;
                            } else if (goonCounter == 2) {
                                state = State.GATE_INTAKES;
                            } else if (goonCounter == 3) {
                                state = State.GATE_INTAKES;
                            } else if (goonCounter == 4) {
                                state = State.GATE_INTAKES;
                            } else if (goonCounter == 5) {
                                state = State.GATE_INTAKES;
                            } else if (goonCounter == 6) {
                                state = State.GATE_INTAKES;
                            } else if (goonCounter == 7) {
                                state = State.OFF_LINE_POS;
                            }

                            goonCounter++;
                        }
                    }
                }
                break;

            case TOP_SPIKE:

                robot.intake.setPower(-1);
                robot.linkage.ON();

                if (splineCounter == 0) {

                    if (dtStallTimer.seconds() == 0) {
                        dtStallTimer.reset();
                    }

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, SPIKE_X_POS, 26, AngleUnit.DEGREES, 270),
                            1, 5, 0.25
                    );

                    if ((robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2)
                            || dtStallTimer.seconds() > 1.0) {

                        splineCounter = 1;
                        dtStallTimer.reset();
                    }

                } else if (splineCounter == 1) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, SPIKE_X_POS, 4, AngleUnit.DEGREES, 270),
                            1, 3, 0.25
                    );

                    if ((robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2)
                            || dtStallTimer.seconds() > 1.0) {

                        splineCounter = 2;
                        dtStallTimer.reset();
                    }
                } else if (splineCounter == 2) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, GATE_HIT_X_POS, 4, AngleUnit.DEGREES, 270),
                            1, 2, 0.25
                    );

                    if (robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 1.4 || dtStallTimer.seconds() > 1.5) {
                        splineCounter = 0;
                        dtStallTimer.reset();
                        state = State.SHOOTING_POSE_1;
                    }
                }

                break;

            case MID_SPIKE:

                robot.intake.setPower(-1);
                robot.linkage.ON();

                if (splineCounter == 0) {

                    if (dtStallTimer.seconds() == 0) {
                        dtStallTimer.reset();
                    }

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, SPIKE_X_POS, 0, AngleUnit.DEGREES, 270),
                            1, 5, 0.25
                    );

                    if ((robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2)
                            || dtStallTimer.seconds() > 1.0) {

                        splineCounter = 1;
                        dtStallTimer.reset();
                    }

                } else if (splineCounter == 1) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, SPIKE_X_POS, -18, AngleUnit.DEGREES, 270),
                            1, 3, 0.25
                    );

                    if ((robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2)
                            || dtStallTimer.seconds() > 1.0) {

                        splineCounter = 2;
                        dtStallTimer.reset();
                    }
                } else if (splineCounter == 2) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, SPIKE_X_POS, 0, AngleUnit.DEGREES, 270),
                            1, 3, 0.25
                    );

                    if ((robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2)
                            || dtStallTimer.seconds() > 1.0) {

                        splineCounter = 3;
                        dtStallTimer.reset();
                    }
                } else if (splineCounter == 3) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, GATE_HIT_X_POS, 0, AngleUnit.DEGREES, 270),
                            1, 2, 0.25
                    );

                    if (robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 1.4 || dtStallTimer.seconds() > 1.5) {
                        splineCounter = 0;
                        dtStallTimer.reset();
                        state = State.SHOOTING_POSE_1;
                    }
                }

                break;

            case GATE_INTAKES:

                robot.intake.setPower(-1);
                robot.linkage.ON();

                if (splineCounter == 0) {

                    if (dtStallTimer.seconds() == 0) {
                        dtStallTimer.reset();
                    }

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, SPIKE_X_POS, -4, AngleUnit.DEGREES, 270),
                            1, 5, 0.25
                    );

                    if ((robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2)
                            || dtStallTimer.seconds() > 1.0) {

                        splineCounter = 1;
                        dtStallTimer.reset();
                    }

                } else if (splineCounter == 1) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, GATE_HIT_X_POS, -4, AngleUnit.DEGREES, 270),
                            1, 2, 0.5
                    );

                    if ((robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2)
                            || dtStallTimer.seconds() > 1.0) {

                        splineCounter = 2;
                        dtStallTimer.reset();
                    }
                } else if (splineCounter == 2) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, SPIKE_X_POS, -16, AngleUnit.DEGREES, 0),
                            1, 3, 0.8
                    );

                    if ((robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2)
                            || dtStallTimer.seconds() > 0.8) {

                        splineCounter = 3;
                        dtStallTimer.reset();
                    }
                } else if (splineCounter == 3) {

                    robot.goToPoint(
                            new Pose2D(DistanceUnit.INCH, 64, -16, AngleUnit.DEGREES, 0),
                            1, 3, 0.25
                    );

                    if (robot.drivetrain.DTatTarget() && dtStallTimer.seconds() > 0.2 || dtStallTimer.seconds() > 1) {
                        splineCounter = 0;
                        dtStallTimer.reset();
                        state = State.SHOOTING_POSE_1;
                    }
                }

                break;

            case OFF_LINE_POS:

                robot.intake.setPower(0);

                robot.goToPoint(
                        new Pose2D(DistanceUnit.INCH, 36, 0, AngleUnit.DEGREES, 270),
                        1, 3, 0.25
                );

                break;
        }
    }
}