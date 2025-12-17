package org.firstinspires.ftc.teamcode.GEN4.Autos;

import static org.firstinspires.ftc.teamcode.GEN4.Autos.Blue15Full.State.FINISHED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;

@Autonomous(name = "Drivetrain Warmup", group = "GEN4")
public class Blue15Full extends LinearOpMode {

    private Robot robot;


    public enum State {
        INITIALIZED,
        START_POSE,
        SHOOTING_POSE_1,
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

    private void updateSequence() {

        switch (state) {

            case INITIALIZED:
                robot.drivetrain.setPosition(-17, -62, 270); // INTAKE FACING DOWN, TURRET SIDE FACING TOWARDS GOAL
                break;

            case SHOOTING_POSE_1:
                robot.update();
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 12, AngleUnit.DEGREES, 240),
                        0.4, 1, 2);
                robot.goalLock(robot.drivetrain.robotPose);


                if (robot.systemsReady()) { robot.rapidFire();
                }
            {
                    if (robot.systemsReady()) { state = FINISHED; }
                    break;

                }
            case FINISHED:
                    break;
        }
    }
}




