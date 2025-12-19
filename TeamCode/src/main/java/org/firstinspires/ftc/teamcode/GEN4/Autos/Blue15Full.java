package org.firstinspires.ftc.teamcode.GEN4.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;

@Autonomous(name = "Drivetrain Warmup", group = "GEN4")
public class Blue15Full extends LinearOpMode {

    private Robot robot;

    private int splineCounter = 0;

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
            robot.update();
            telemetry.addData("AUTO FSM", state);
            telemetry.addData("DRIVETRAIN FSM", robot.drivetrain.state);
            telemetry.addData("Robot X", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("Robot Y", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("Robot Heading", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));

        }

        waitForStart();

        state = State.SHOOTING_POSE_1; // FIRST POINT AFTER PRESSING "START"
        while (opModeIsActive()) {
            updateSequence();

            // DETAILED TELEMETRY
            telemetry.addData("AUTO FSM", state);
            telemetry.addData("DRIVETRAIN FSM", robot.drivetrain.state);
            telemetry.addData("Robot X", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("Robot Y", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("Robot Heading", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("At Position", robot.drivetrain.DTatTarget());
            telemetry.addData("Wait Active", robot.wait.isActive());
            telemetry.addData("Wait Finished", robot.wait.isFinished());
            telemetry.update();
        }
    }

    boolean atPosition = false;

    private void nextSplinePoint() {
        if (robot.drivetrain.DTatTarget()) { splineCounter ++; };
    }

    private void updateSequence() {

        switch (state) {

            case INITIALIZED:
                robot.drivetrain.setPosition(-17, -62, 90); // INTAKE FACING DOWN, TURRET SIDE FACING TOWARDS GOAL
                robot.readMotifTag();
                telemetry.addData("MOTIF:", robot.MotifTagID);
                telemetry.update();
                break;

            case SHOOTING_POSE_1:
                robot.update();
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, -50, AngleUnit.DEGREES, 240),
                        0.7, 2, 2);
                robot.goalLock(robot.drivetrain.robotPose);

                if (robot.systemsReady()) { robot.rapidFire(); }
                if (!robot.colors.hasBall) { state = State.FIRST_INTAKES; }
                break;

            case FIRST_INTAKES:

                robot.update();
                if (splineCounter == 0) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -50, -48, AngleUnit.DEGREES, 260),
                            0.4, 3, 3);
                    nextSplinePoint();
                } else if (splineCounter == 1) {
                    robot.intake.runIntake(-1.0); // INTAKE CORNER BALLS

                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -50, -60, AngleUnit.DEGREES, 260),
                            0.4, 3, 3);
                    nextSplinePoint();
                } else if (splineCounter == 2) {
                    robot.intake.runIntake(-0.4); // KEEP RUNNING INTAKE TO ENSURE BALLS IN
                    robot.goalLock(robot.drivetrain.robotPose);

                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, -50, AngleUnit.DEGREES, 270),
                            0.4, 3, 3);
                    nextSplinePoint();
                } else if (splineCounter == 3) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, -50, AngleUnit.DEGREES, 270),
                            0.4, 3, 3);
                    nextSplinePoint();
                }


                if (robot.drivetrain.DTatTarget()) { state = State.POINT_3; }
                break;

            case POINT_3:
                robot.update();
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -40, 12, AngleUnit.DEGREES, 180),
                        0.4, 3, 3);
                robot.intake.runIntake(-1.0);
                if (robot.drivetrain.DTatTarget()) { state = State.POINT_3; }
                break;

            case FINISHED:
                    break;
        }
    }
}




