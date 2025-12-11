package org.firstinspires.ftc.teamcode.GEN4.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;

@TeleOp(name = "GEN4 TeleOp", group = "GEN4")
public class GEN4Teleop extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.startup();
        robot.drivetrain.state = Drivetrain.State.TELEOP;

        robot.arms.arm1_flickOFF();
        robot.arms.arm2_flickOFF();
        robot.arms.arm3_flickOFF();
    }

    @Override
    public void loop() {

        // ---------- DRIVING ----------
        double drive  = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn   = -gamepad1.right_stick_x;

        if (gamepad1.right_trigger > 0.01) {
            drive  *= 0.5;
            strafe *= 0.5;
            turn   *= 0.5;
        }
        robot.drivetrain.driveTeleOp(strafe, drive, turn);

        // ---------- ARMS & INTAKE ----------
        double intakePower = 0;
        if (gamepad1.right_bumper) {
            robot.arms.arm3_flickON();
            intakePower = -0.5;
        } else {
            robot.arms.arm1_flickOFF();
            robot.arms.arm2_flickOFF();
            robot.arms.arm3_flickOFF();
        }
        robot.intake.runIntake(intakePower);

        // ---------- GOAL TRACKING ----------
        Pose2D goal = robot.getTargetGoal();
        robot.flywheel.aimToGoal(goal, robot.drivetrain.robotPose,
                robot.drivetrain.XVel(), robot.drivetrain.YVel());
        robot.differential.aimToGoal(robot.drivetrain.robotPose, goal);

        // ---------- UPDATE ----------
        robot.update();

        // ---------- TELEMETRY ----------
        telemetry.addLine("=== ROBOT ===");
        telemetry.addData("X/Y/Heading", "%.1f / %.1f / %.1f",
                robot.drivetrain.robotPose.getX(DistanceUnit.INCH),
                robot.drivetrain.robotPose.getY(DistanceUnit.INCH),
                robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Drive State", robot.drivetrain.state);

        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("Power", "%.2f", intakePower);
        telemetry.addData("Current L/R", "%.2f / %.2f", robot.intake.getCurrentL(), robot.intake.getCurrentR());

        telemetry.addLine("=== FLYWHEEL ===");
        telemetry.addData("Target/Actual (rad/s)", "%.1f / %.1f",
                robot.flywheel.getTargetVelocity(), robot.flywheel.getVelocityRadPerSec());
        telemetry.addData("Target/Actual (RPM)", "%.1f / %.1f",
                robot.flywheel.getTargetVelocity() * 60 / (2*Math.PI),
                robot.flywheel.getVelocityRPM());

        telemetry.addLine("=== DIFFERENTIAL ===");
        telemetry.addData("Encoders L/R", "%d / %d",
                robot.differential.encL.getCurrentPosition(),
                robot.differential.encR.getCurrentPosition());
        telemetry.addData("Target L/R", "%.1f / %.1f",
                -robot.differential.targetL, robot.differential.targetR);
        telemetry.addData("TARGET DIFFY ANGLE: ", robot.differential.compensatedAngle);

        double dx = goal.getX(DistanceUnit.INCH) - robot.drivetrain.robotPose.getX(DistanceUnit.INCH);
        double dy = goal.getY(DistanceUnit.INCH) - robot.drivetrain.robotPose.getY(DistanceUnit.INCH);
        telemetry.addData("Distance to Goal", "%.2f", Math.hypot(dx, dy));

        telemetry.update();
    }
}
