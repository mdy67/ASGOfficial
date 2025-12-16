package org.firstinspires.ftc.teamcode.GEN4.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.alliance;

@Autonomous(name = "Drivetrain Warmup", group = "GEN4")
public class Blue15Full extends LinearOpMode {

    private Robot robot;


    public enum State {
        INITIALIZED,
        START_POSE,
        POINT_1,
        POINT_2,
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

        state = State.POINT_1; // FIRST POINT AFTER PRESSING "START"
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
    private void updateSequence() {

        switch (state) {

            case INITIALIZED:
                robot.drivetrain.setPosition(-17, -62, 270); // INTAKE FACING DOWN, TURRET SIDE FACING TOWARDS GOAL
                break;

            case POINT_1:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 12, AngleUnit.DEGREES, 240),
                        0.4, 1, 2);
                robot.goalLock();
                robot.update();


                atPosition = robot.drivetrain.DTatTarget();
                if (atPosition && robot.differential.atTarget && robot.flywheel.atTargetVelocity()) { state = State.POINT_2; }
                break;

            case POINT_2:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 12, AngleUnit.DEGREES, 180),
                        0.25, 4, 5);
                robot.update();
                atPosition = robot.drivetrain.DTatTarget();
                if (atPosition) { state = State.POINT_3; }
                break;

            case POINT_3:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -49, 12, AngleUnit.DEGREES, 180),
                        0.25, 1, 5);
                robot.update();
                atPosition = robot.drivetrain.DTatTarget();
                if (atPosition) { state = State.POINT_4; }
                break;

            case POINT_4:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 12, AngleUnit.DEGREES, 240),
                        0.4, 3, 5);
                robot.update();
                atPosition = robot.drivetrain.DTatTarget();
                if (atPosition) { state = State.POINT_5; }
                break;

            case POINT_5:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, -12, AngleUnit.DEGREES, 180),
                        0.4, 3, 5);
                robot.update();
                atPosition = robot.drivetrain.DTatTarget();
                if (atPosition) { state = State.POINT_6; }
                break;

            case POINT_6:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -49, -12, AngleUnit.DEGREES, 180),
                        0.4, 3, 5);
                robot.update();
                atPosition = robot.drivetrain.DTatTarget();
                if (atPosition) { state = State.POINT_7; }
                break;
            case POINT_7:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 12, AngleUnit.DEGREES, 240),
                        0.4, 1, 2);
                robot.update();
                atPosition = robot.drivetrain.DTatTarget();
                if (atPosition) { state = State.POINT_8; }
                break;
            case POINT_8:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -17, -20, AngleUnit.DEGREES, 270),
                        0.4, 8, 5);
                robot.update();
                atPosition = robot.drivetrain.DTatTarget();
                if (atPosition) { state = State.POINT_9; }
                break;
            case POINT_9:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -17, -62, AngleUnit.DEGREES, 270),
                        0.1, 1, 2);
                robot.update();
                atPosition = robot.drivetrain.DTatTarget();
                if (atPosition) { state = State.FINISHED; }
                break;

            case FINISHED:
                robot.update();
                robot.drivetrain.state = Drivetrain.State.IDLE;
                break;
        }
    }
}

