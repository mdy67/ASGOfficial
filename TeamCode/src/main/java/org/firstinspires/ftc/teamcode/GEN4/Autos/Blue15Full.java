package org.firstinspires.ftc.teamcode.GEN4.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Wait;

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

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
        robot.startup();

        // INIT LOOoOP
        while (opModeInInit()) {
            state = State.INITIALIZED;
            updateSequence();
            telemetry.addData("AUTO FSM", state);
            telemetry.addData("DRIVETRAIN FSM", robot.drivetrain.state);
            telemetry.addData("Robot X", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("Robot Y", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("Robot Heading", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));

        }

        waitForStart();

        state = State.SHOOTING_POSE_1; // FIRST POINT AFTER PRESSING "START"
        while (opModeIsActive() && !state.equals(State.FINISHED)) {
            updateSequence();

            // DETAILED TELEMETRY
            telemetry.addData("AUTO FSM", state);
            telemetry.addData("DRIVETRAIN FSM", robot.drivetrain.state);
            telemetry.addData("Robot X", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("Robot Y", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("Robot Heading", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("At Position", robot.drivetrain.DTatTarget());
            telemetry.update();
        }
    }

    boolean atPosition = false;


    private void updateSequence() {

        switch (state) {

            case INITIALIZED:
                robot.update();
                robot.drivetrain.setPosition(-16, -66, 90); // INTAKE FACING DOWN, TURRET SIDE FACING TOWARDS GOAL
                robot.neutral();
                robot.readMotifTag();
                telemetry.addData("MOTIF:", robot.MotifTagID);
                telemetry.update();
                break;

            case SHOOTING_POSE_1: // SHOOT PRELOADS [NON SORTED]

                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, -55, AngleUnit.DEGREES, 240),
                        0.25, 1, 1);
                robot.update();
              //  robot.goalLock(robot.drivetrain.robotPose);

             //   if (robot.systemsReady()) { robot.rapidFire(); }
                if (!robot.colors.hasBall && robot.drivetrain.DTatTarget()) { state = State.FIRST_INTAKES; }
                break;

            case FIRST_INTAKES: // INTAKE PGP CORNER BALLS, SHOOT CLOSE ZONE

                if (robot.splineCounter == 0) { // CORNER ENTERING POS

                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -30, -53, AngleUnit.DEGREES, 210),
                            1, 10, 30);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 1) { // CORNER ENTERING POS
                    robot.intake.runIntake(-1.0); // INTAKE CORNER BALLS
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -59, -50, AngleUnit.DEGREES, 240),
                            1, 4, 8);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 2) { // DEEP CORNER POS
                    robot.intake.runIntake(-1.0); // INTAKE CORNER BALLS

                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -60, -58, AngleUnit.DEGREES, 250),
                            1, 3, 8);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 3) {
                    robot.intake.runIntake(-1.0); // INTAKE CORNER BALLS

                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -60, -65, AngleUnit.DEGREES, 270),
                            1, 3, 3);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 4) {
                    robot.intake.runIntake(-1.0);

                 //   robot.goalLock(robot.drivetrain.robotPose);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -28, -55, AngleUnit.DEGREES, 260),
                            1, 7, 80);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 5) { // SHOOTING BALLS 4, 5, 6 POS
                    robot.intake.runIntake(0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 10, AngleUnit.DEGREES, 260),
                            1, 4, 4);

                //    if (robot.systemsReady()) { robot.rapidFire(); }

                    robot.update();
                    if (robot.drivetrain.DTatTarget()) {
                        robot.intake.runIntake(1.0);
                    }
                    if (!robot.colors.hasBall) {
                        robot.nextSplinePoint(); // SHOOT BALLS POS
                    }
                }

                if (robot.splineCounter == 6) { robot.resetSplineCounter(); state = State.POINT_3; } break;

            case POINT_3: // INTAKE MIDDLE BALLS

                if (robot.splineCounter == 0) { //

                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -24, -10, AngleUnit.DEGREES, 180),
                            1, 2, 2);
                    robot.nextSplinePoint();

                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -55, -14, AngleUnit.DEGREES, 180),
                            1, 2, 10);
                    robot.nextSplinePoint();

                } else if (robot.splineCounter == 2) {
                    robot.intake.runIntake(-1.0); //
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -48, -6, AngleUnit.DEGREES, 180),
                            1, 2, 10);
                    robot.nextSplinePoint();

                } else if (robot.splineCounter == 3) {
                    robot.intake.runIntake(0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -52, -6, AngleUnit.DEGREES, 180),
                            1, 2, 3);
                    robot.nextSplinePoint();
                } else if (robot.splineCounter == 4) {
                    robot.intake.runIntake(0); //
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -26, 0, AngleUnit.DEGREES, 180),
                    1, 4, 10);
                    robot.nextSplinePoint();

                } else if (robot.splineCounter == 5) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 10, AngleUnit.DEGREES, 260),
                        1, 4, 4);

                    robot.update();
                    if (robot.drivetrain.DTatTarget()) {
                        robot.intake.runIntake(1.0); // TODO: REPLACE WITH SHOOTING ALG
                    }
                    if (!robot.colors.hasBall) {
                        robot.nextSplinePoint();
                    }
                }

                if (robot.splineCounter == 6) { robot.resetSplineCounter(); state = State.POINT_4; } break;

            case POINT_4: // INTAKE BOTTOM BALLS

                if (robot.splineCounter == 0) { //

                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -24, -38, AngleUnit.DEGREES, 180),
                            1, 5, 10);
                    robot.nextSplinePoint();

                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -55, -38, AngleUnit.DEGREES, 180),
                            1, 4, 10);
                    robot.nextSplinePoint();
                    if (robot.splineCounter == 2) {
                        robot.intake.runIntake(0);
                    }

                } else if (robot.splineCounter == 2) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -28, -15, AngleUnit.DEGREES, 260),
                    1, 15, 80);
                    robot.nextSplinePoint();

                } else if (robot.splineCounter == 3) {

                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 10, AngleUnit.DEGREES, 260),
                            1, 4, 4);

                    robot.update();
                    if (robot.drivetrain.DTatTarget()) {
                        robot.intake.runIntake(1.0); // TODO: REPLACE WITH SHOOTING ALG
                    }
                    if (!robot.colors.hasBall) {
                        robot.nextSplinePoint();
                    }
                }

                if (robot.splineCounter == 4) { robot.resetSplineCounter(); state = State.POINT_5; } break;

            case POINT_5: // INTAKE BOTTOM BALLS

                if (robot.splineCounter == 0) { //

                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -24, 12, AngleUnit.DEGREES, 180),
                            1, 5, 5);
                    robot.nextSplinePoint();

                } else if (robot.splineCounter == 1) {
                    robot.intake.runIntake(-1.0);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -50, 12, AngleUnit.DEGREES, 180),
                            1, 2, 10);
                    robot.nextSplinePoint();

                } else if (robot.splineCounter == 2) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 10, AngleUnit.DEGREES, 260),
                            1, 15, 80);

                    robot.update();
                    if (robot.drivetrain.DTatTarget()) {
                        robot.intake.runIntake(1.0); // TODO: REPLACE WITH SHOOTING ALG
                    }
                    if (!robot.colors.hasBall) {
                        robot.nextSplinePoint();
                    }
                }

                if (robot.splineCounter == 3) { robot.resetSplineCounter(); state = State.FINISHED; } break;

            case FINISHED:
                // ENDS OPMODE LOOP
                    break;
        }
    }
}




