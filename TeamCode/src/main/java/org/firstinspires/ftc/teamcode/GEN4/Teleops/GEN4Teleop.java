package org.firstinspires.ftc.teamcode.GEN4.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.AutoToTeleop;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.alliance;

@TeleOp(name = "GEN4 TeleOp", group = "GEN4")
public class GEN4Teleop extends OpMode {

    private Robot robot;
    private boolean TUNING_MODE = false;
    private double targetVel = 350;
    private double hoodAngle = 0.4;
    private double modifierLive = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        timer.reset();
        robot = new Robot(hardwareMap);
        robot.arms.reset();
      //  alliance.set(alliance.Color.BLUE);
        robot.drivetrain.state = Drivetrain.State.TELEOP;
        robot.importAutoPose(AutoToTeleop.storedPose.getX(DistanceUnit.INCH), AutoToTeleop.storedPose.getY(DistanceUnit.INCH), AutoToTeleop.storedPose.getHeading(AngleUnit.DEGREES));
     //   robot.importAutoDiffy(AutoToTeleop.encLOffset, AutoToTeleop.encROffset);
        robot.flywheel.stop();

        robot.flywheel.MAX_VELOCITY = 450;
        robot.differential.farZone = false;
        robot.flywheel.velocityModifier = -55 + modifierLive;

        telemetry.addLine("Robot initialized. Waiting for start...");
        telemetry.update();


    }



    @Override
    public void loop() {

        /**
         *
         * LEFT TRIGGER = OUTTAKE
         * LEFT BUMPER = INTAKE
         *
         * RIGHT TRIGGER = SLOW MODE
         * RIGHT BUMPER = RAPID FIRE
         *
         *
         */



        double drive  = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn   = -gamepad1.right_stick_x;

        if (gamepad1.right_trigger > 0.01) {
            drive  *= 0.4;
            strafe *= 0.4;
            turn   *= 0.4;
        }

        robot.drivetrain.driveTeleOp(strafe, drive, turn);

        // Intake & Arms
        double intakePower = 0;
        if (gamepad1.left_bumper) {
            intakePower = -1.0;
        } else if (gamepad1.a) {
            intakePower = 0.75;
        }

        /**
         *
         *             Y
         *        X    :     B
         *             A
         *
         */


        if (gamepad1.x) {
            robot.arms.arm3_flickON();
            intakePower = -0.8;
        }
        if (gamepad1.y) {
            robot.arms.arm2_flickON();
        } else {
            robot.arms.arm2_flickOFF();
        }
        if (gamepad1.b) {
            robot.arms.arm1_flickON();
        } else {
            robot.arms.arm1_flickOFF();
        }
        if (gamepad1.right_bumper) {
            robot.arms.arm3_flickRAPID();
            intakePower = -0.8;
        }

        if (!gamepad1.x && !gamepad1.right_bumper) { robot.arms.arm3_flickOFF(); }

        robot.intake.runIntake(intakePower); // APPLY INTAKEPOWER intakePower

        // Turret / Goal Tracking
        Pose2D goal = robot.getTargetGoal();
        double dx = goal.getX(DistanceUnit.INCH) - robot.drivetrain.robotPose.getX(DistanceUnit.INCH);
        double dy = goal.getY(DistanceUnit.INCH) - robot.drivetrain.robotPose.getY(DistanceUnit.INCH);
        double distance = Math.hypot(dx, dy);

        robot.flywheel.MAX_VELOCITY = 450;
        robot.differential.farZone = false;
        robot.flywheel.velocityModifier = -40.0 + modifierLive;
        /*
        if (gamepad1.left_stick_button) { // UP = CLOSE ZONE

            robot.flywheel.velocityModifier = -40.0 + modifierLive;

        } else if (gamepad1.right_stick_button) {
            robot.flywheel.MAX_VELOCITY = 550; // LEFT = FAR ZONE
            // TODO: IF YOU CHANGE THIS THEN CHANGE LINES BELOW THAT USE THIS

            robot.differential.farZone = true;
            robot.flywheel.velocityModifier = 30.0 + modifierLive;
        }

         */

        if (robot.drivetrain.robotPose.getY(DistanceUnit.INCH) > -24) {
            robot.differential.SPECIAL_OFFSET = robot.drivetrain.robotPose.getY(DistanceUnit.INCH) * (2.0/60.0); // 2/60 [2 DEGREES AT 60 INCHES]
        } else {
            robot.differential.SPECIAL_OFFSET = 0;
        }


        if (TUNING_MODE) {
         //   if (gamepad1.dpad_left)  targetVel -= 2;
         //   if (gamepad1.dpad_right) targetVel += 2;
            robot.flywheel.setTargetVelocity(targetVel);
            robot.flywheel.angleHood(targetVel);
            robot.differential.setTargetAngle(90);

          //  if (gamepad1.x) hoodAngle += 0.01;
          //  if (gamepad1.y) hoodAngle -= 0.01;
         //   robot.flywheel.setHoodAngle(hoodAngle);
        } else {
            robot.flywheel.aimToGoal(robot.getTargetGoal(), robot.drivetrain.robotPose, robot.drivetrain.XVel(), robot.drivetrain.YVel());
            if (gamepad1.left_trigger > 0.1) {
                robot.differential.aimToGoal(robot.drivetrain.robotPose, robot.getTargetGoal());
            }

        }

        if (timer.seconds() < 10) {
            robot.differential.aimToGoal(robot.drivetrain.robotPose, robot.getTargetGoal());
        }

        if (gamepad1.dpad_right) {
            robot.differential.ANGLE_ADJUST -= 2.5;
        } else if (gamepad1.dpad_left) {
            robot.differential.ANGLE_ADJUST += 2.5;
        }





        if (gamepad1.dpad_up) {
            modifierLive += 5;
        } else if (gamepad1.dpad_down) {
            modifierLive -= 5;
        }


        robot.update();

        telemetry.addLine("=== ROBOT POSE ===");
        telemetry.addData("X / Y / Heading", "%.1f / %.1f / %.1f",
                robot.drivetrain.robotPose.getX(DistanceUnit.INCH),
                robot.drivetrain.robotPose.getY(DistanceUnit.INCH),
                robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("=== ADJ POSE ===");
        telemetry.addData("X / Y / Heading", "%.1f / %.1f / %.1f",
                robot.adjustedPose.getX(DistanceUnit.INCH),
                robot.adjustedPose.getY(DistanceUnit.INCH),
                robot.adjustedPose.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("=== LL POSE ===");
        telemetry.addData("X / Y / Heading", "%.1f / %.1f / %.1f",
                robot.limelight.LimelightPose.position.x,
                robot.limelight.LimelightPose.position.y,
                robot.adjustedPose.getHeading(AngleUnit.DEGREES));

      //  telemetry.addLine("=== DIFFERENTIAL ===");
     //   telemetry.addData("Encoders L/R", "%d / %d",
         //       robot.differential.getEncoderL(),
        //        robot.differential.getEncoderR());
      //  telemetry.addData("Target L/R", "%.1f / %.1f",
         //       -robot.differential.targetL,
           //     robot.differential.targetR);
      //  telemetry.addData("Target Angle", "%.1f", robot.differential.compensatedAngle);
      //  telemetry.addData("Desired Angle", "%.1f", robot.differential.desiredAngle);
        telemetry.addLine();

        telemetry.addData("Distance to Goal", "%.2f", distance);
        telemetry.addData("FLYWHEEL TARGET: ", robot.flywheel.getTargetVelocity());
        telemetry.addData("CURRENT VELOCITY: ", robot.flywheel.getVelocityRadPerSec());

        telemetry.addLine();

        telemetry.addData("Target L: ", robot.differential.targetL);
        telemetry.addData("Target R: ", robot.differential.targetR);
        telemetry.addData("Desired Angle: ", robot.differential.desiredAngle);
        telemetry.addData("Compensated Angle: ", robot.differential.compensatedAngle);
        telemetry.addData("Gantry Slot: ", robot.differential.currentSlotBase);
        telemetry.update();
    }
}
