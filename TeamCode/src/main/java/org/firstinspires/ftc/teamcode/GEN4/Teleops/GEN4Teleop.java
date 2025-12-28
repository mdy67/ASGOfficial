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
 //   double targetVel = 200;
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.startup();
        robot.drivetrain.state = Drivetrain.State.TELEOP;

    }
    double targetVel = 200;
    double hoodAngle = 0.4;
    private boolean TUNING_MODE = false;
    @Override
    public void loop() {

        // ---------- DRIVING ----------
        double drive  = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn   = -gamepad1.right_stick_x;

        if (gamepad1.right_trigger > 0.01) {
            drive  *= 0.4;
            strafe *= 0.4;
            turn   *= 0.4;
        }

        robot.drivetrain.driveTeleOp(strafe, drive, turn);

        // ---------- ARMS & INTAKE ----------
        double intakePower = 0;

        if (gamepad1.right_bumper) {
            robot.arms.arm3_flickRAPID();
            intakePower = -0.8; // RAPID FIRE
        }
        else if (gamepad1.left_bumper) {
            intakePower = -1.0; // INTAKE
        }
        else if (gamepad1.dpad_down) {
            intakePower = 0.75; // OUTTAKE
        }
        else if (gamepad1.a) {
            robot.arms.arm3_flickON();
            intakePower = -0.8; // STANDARD FIRE
        }
        else {
            robot.arms.arm1_flickOFF();
            robot.arms.arm2_flickOFF();
            robot.arms.arm3_flickOFF();
        }

      //  if (gamepad1.dpad_right) {
     //       robot.differential.ANGLE_ADJUST --;
     //   } else if (gamepad1.dpad_left) {
     //       robot.differential.ANGLE_ADJUST ++;
     //   }

        robot.intake.runIntake(intakePower);

        // ---------- GOAL TRACKING ----------
        Pose2D goal = robot.getTargetGoal();

        if (TUNING_MODE) {
            if (gamepad1.dpad_left) { targetVel -= 2; }
            if (gamepad1.dpad_right) { targetVel += 2; }
            robot.flywheel.setTargetVelocity(targetVel);

            if (gamepad1.x) { hoodAngle += 0.01; }
            if (gamepad1.y) { hoodAngle -= 0.01; }
            robot.flywheel.setHoodAngle(hoodAngle);
        } else {
            robot.goalLock(robot.adjustedPose);
        }



        if (robot.differential.atTarget && robot.colors.hasBall) {
            gamepad1.rumble(100);
        }


        // ---------- UPDATE ----------
        robot.update();

        // ---------- TELEMETRY ----------
        telemetry.addLine("=== ROBOT POSE ===");
        telemetry.addData("X / Y / Heading", "%.1f / %.1f / %.1f",
                robot.drivetrain.robotPose.getX(DistanceUnit.INCH),
                robot.drivetrain.robotPose.getY(DistanceUnit.INCH),
                robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));

        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("Power", "%.2f", intakePower);
        telemetry.addData("Current L/R", "%.2f / %.2f",
                robot.intake.getCurrentL(),
                robot.intake.getCurrentR());

        telemetry.addLine("=== FLYWHEEL ===");
        telemetry.addData("Target/Actual (rad/s)", "%.1f / %.1f",
                robot.flywheel.getTargetVelocity(),
                robot.flywheel.getVelocityRadPerSec());
        telemetry.addData("Target/Actual (RPM)", "%.1f / %.1f",
                robot.flywheel.getTargetVelocity() * 60 / (2 * Math.PI),
                robot.flywheel.getVelocityRPM());

        telemetry.addLine("=== DIFFERENTIAL ===");
        telemetry.addData("Encoders L/R", "%d / %d",
                robot.differential.encL.getCurrentPosition(),
                robot.differential.encR.getCurrentPosition());
        telemetry.addData("Target L/R", "%.1f / %.1f",
                -robot.differential.targetL,
                robot.differential.targetR);
        telemetry.addData("Target Angle", "%.1f",
                robot.differential.compensatedAngle);

        double dx = goal.getX(DistanceUnit.INCH)
                - robot.drivetrain.robotPose.getX(DistanceUnit.INCH);
        double dy = goal.getY(DistanceUnit.INCH)
                - robot.drivetrain.robotPose.getY(DistanceUnit.INCH);



        // ---------- LIMELIGHT DEBUG (SAFE) ----------
        telemetry.addLine("=== LIMELIGHT ===");
        telemetry.addData("Pose", "%.1f / %.1f / %.1f",
                robot.limelight.LimelightPose.position.x,
                robot.limelight.LimelightPose.position.y,
                robot.limelight.LimelightPose.heading.toDouble());

        telemetry.addData("Adjusted Pose", "%.1f / %.1f / %.1f",
                robot.adjustedPose.getX(DistanceUnit.INCH),
                robot.adjustedPose.getY(DistanceUnit.INCH),
                robot.adjustedPose.getHeading(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addData("FLYWHEEL VELOCITY:", targetVel);
        telemetry.addData("HOOD ANGLE:", robot.flywheel.getHoodAngle());
        telemetry.addData("Distance to Goal", "%.2f", Math.hypot(dx, dy));
        telemetry.update();
    }
}
