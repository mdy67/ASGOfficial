package org.firstinspires.ftc.teamcode.GEN4.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Differential;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;

@TeleOp(name = "GEN4 TeleOp", group = "GEN4")
public class GEN4Teleop extends OpMode {

    private Robot robot;
  //  private boolean differentialInitialized = false;
    private boolean TUNING_MODE = false;
    private double targetVel = 200;
    private double hoodAngle = 0.4;
   // double targetAngle;
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.startup();
        robot.drivetrain.state = Drivetrain.State.TELEOP;

        telemetry.addLine("Robot initialized. Waiting for start...");
        telemetry.update();
    }

    @Override
    public void loop() {

        // --------------------
        // Initialize differential once
        // --------------------
     //   if (!differentialInitialized) {
      //      robot.differential.resetToSlot3();
      //      differentialInitialized = true;
     //   }

        // --------------------
        // Driving
        // --------------------
        double drive  = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn   = -gamepad1.right_stick_x;

        // Precision mode
        if (gamepad1.right_trigger > 0.01) {
            drive  *= 0.4;
            strafe *= 0.4;
            turn   *= 0.4;
        }

        robot.drivetrain.driveTeleOp(strafe, drive, turn);

        // --------------------
        // Intake & Arms
        // --------------------
        double intakePower = 0;

        if (gamepad1.right_bumper) {
            robot.arms.arm3_flickRAPID();
            intakePower = -0.8; // RAPID FIRE
        } else if (gamepad1.left_bumper) {
            intakePower = -1.0; // INTAKE
        } else if (gamepad1.dpad_down) {
            intakePower = 0.75; // OUTTAKE
        } else if (gamepad1.a) {
            robot.arms.arm3_flickON();
            intakePower = -0.8; // STANDARD FIRE
        } else {
            robot.arms.reset();
        }

        robot.intake.runIntake(intakePower);

        // --------------------
        // Turret / Goal Tracking
        // --------------------
        Pose2D goal = robot.getTargetGoal();
        robot.goalLock(robot.drivetrain.robotPose);
           /*
        int currentSlot;
        if (gamepad1.dpad_up) {
            currentSlot = 1;
            robot.differential.goToSlot(currentSlot);

        } else if (gamepad1.dpad_left) {
            currentSlot = 2;
            robot.differential.goToSlot(currentSlot);

        } else if (gamepad1.dpad_down) {
            currentSlot = 3;
            robot.differential.goToSlot(currentSlot);

        }

        // -------------------------
        // Angle adjustment
        // -------------------------

         if (gamepad1.left_bumper) {
            targetAngle -= 0.5; // small step
            robot.differential.setTargetAngle(targetAngle);
        } else if (gamepad1.right_bumper) {
            targetAngle += 0.5;
            robot.differential.setTargetAngle(targetAngle);
        }
         */


        // -------------------------
        // Run update
        // -------------------------
        //differential.update();

        // --------------------
        // Tuning mode for flywheel/hood
        // --------------------
        if (TUNING_MODE) {
            if (gamepad1.dpad_left)  targetVel -= 2;
            if (gamepad1.dpad_right) targetVel += 2;
            robot.flywheel.setTargetVelocity(targetVel);

            if (gamepad1.x) hoodAngle += 0.01;
            if (gamepad1.y) hoodAngle -= 0.01;
            robot.flywheel.setHoodAngle(hoodAngle);
        }

        // --------------------
        // Update all systems
        // --------------------
        robot.update();

        // --------------------
        // Telemetry
        // --------------------
        telemetry.addLine("=== ROBOT POSE ===");
        telemetry.addData("X / Y / Heading", "%.1f / %.1f / %.1f",
                robot.drivetrain.robotPose.getX(DistanceUnit.INCH),
                robot.drivetrain.robotPose.getY(DistanceUnit.INCH),
                robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));

        telemetry.addLine("=== DIFFERENTIAL ===");
        telemetry.addData("Encoders L/R", "%d / %d",
                robot.differential.getEncoderL(),
                robot.differential.getEncoderR());
        telemetry.addData("Target L/R", "%.1f / %.1f",
                -robot.differential.targetL,
                robot.differential.targetR);
        telemetry.addData("Target Angle", "%.1f", robot.differential.compensatedAngle);
        telemetry.addData("Desired Angle", "%.1f", robot.differential.desiredAngle);

        double dx = goal.getX(DistanceUnit.INCH) - robot.drivetrain.robotPose.getX(DistanceUnit.INCH);
        double dy = goal.getY(DistanceUnit.INCH) - robot.drivetrain.robotPose.getY(DistanceUnit.INCH);
        telemetry.addData("Distance to Goal", "%.2f", Math.hypot(dx, dy));

        telemetry.update();
    }
}
