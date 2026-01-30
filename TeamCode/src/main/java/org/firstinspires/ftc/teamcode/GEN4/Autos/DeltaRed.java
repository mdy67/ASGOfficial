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

@Autonomous(name = "Delta Red", group = "GEN4")
public class DeltaRed extends LinearOpMode {

    private Robot robot;

    public enum State {
        INITIALIZED,
        START_POSE,
        SHOOTING_POSE_1,
        FIRST_INTAKES,
        SHOOTING_POSE_2,
        POOL_INTAKE,
        POINT_5,
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

        robot.flywheel.MAX_VELOCITY = 800;
        alliance.set(alliance.Color.BLUE);

        while (opModeInInit()) {
            state = State.INITIALIZED;
            robot.flywheel.stop();
            updateSequence();
        }

        waitForStart();
        state = State.SHOOTING_POSE_1;

        while (opModeIsActive() && !state.equals(State.FINISHED)) {
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

            /* ================= SHOOT 1 ================= */

            case SHOOTING_POSE_1:
                robot.goalLock(robot.drivetrain.robotPose);
                robot.differential.farZone = true;

                if (!trigger) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, 18, -57, AngleUnit.DEGREES, 300), 1, 3, 0.2);
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

            case FIRST_INTAKES:
                if (robot.splineCounter == 0) {
                    dtStallTimer.reset();
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, 24, -38, AngleUnit.DEGREES, 0), 1, 3, 0.3);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, 66, -38, AngleUnit.DEGREES, 0), 1, 4, 10);
                    robot.nextSplinePoint();
                    checkStallTimer(2);
                } else if (robot.splineCounter == 2) {
                    robot.resetSplineCounter();
                    state = State.FIRST_INTAKES;
                }
                break;

            case SHOOTING_POSE_2:
                robot.goalLock(robot.drivetrain.robotPose);
                robot.differential.farZone = true;

                if (!trigger) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, 18, -57, AngleUnit.DEGREES, 300), 1, 3, 0.2);
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
                        dtStallTimer.reset();
                        CycleCounter ++;
                        if (CycleCounter > 2) {
                            state = State.POINT_5;
                        } else {
                            state = State.POOL_INTAKE;
                        }

                        trigger = false;
                    }
                }

                robot.update();
                if (robot.drivetrain.DTatTarget()) {
                    trigger = true;
                    robot.drivetrain.state = Drivetrain.State.IDLE;
                }
                break;

            case POOL_INTAKE:
                if (robot.splineCounter == 0) {
                    checkStallTimer(3);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, 58, -30, AngleUnit.DEGREES, 300), 1, 3, 0.3);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, 60, -38, AngleUnit.DEGREES, 300), 1, 4, 10);
                    robot.nextSplinePoint();
                    checkStallTimer(2);
                } else if (robot.splineCounter == 2) {
                    robot.resetSplineCounter();
                    state = State.SHOOTING_POSE_2;
                }
                break;

            case POINT_5: // MOVE OFF LINE
                if (robot.splineCounter == 0) {
                    checkStallTimer(3);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, 18, -40, AngleUnit.DEGREES, 300), 0.3, 3, 0.3);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) {
                    state = State.FINISHED;
                }
                break;

            case FINISHED:
                robot.update();
                break;
        }
    }

    public void checkStallTimer(double threshold) {
        if (dtStallTimer.seconds() > threshold) {
            robot.splineCounter++;
            dtStallTimer.reset();
        }
    }
}




