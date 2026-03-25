package org.firstinspires.ftc.teamcode.VIPER.TELEOPs;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.VIPER.Robot;
import org.firstinspires.ftc.teamcode.VIPER.alliance;

@TeleOp(name = "VIPER TeleOp", group = "GEN4")
public class VIPERTELEOP extends OpMode {

    private Robot robot;

    double targetVel = 0;
    double targetIntakeVel = 0;
    double tempHoodAngle = 0;
    double kTime = 0.41;

    boolean trigger = false;
    ElapsedTime timer;

    double targetTurretAngle = 180;

    @Override
    public void init() {
        timer = new ElapsedTime();
        robot = new Robot(hardwareMap);

        // === DASHBOARD TELEMETRY (ADDED) ===
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25); // smoother live graphs

        robot.update(3); // 0 = IDLE
        telemetry.addLine("Robot initialized. Waiting for start...");
        telemetry.update();

        alliance.set(alliance.Color.BLUE);

    }


    @Override
    public void loop() {


        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        if (gamepad1.right_trigger > 0.01) {
            drive *= 0.4;
            strafe *= 0.4;
            turn *= 0.4;
        }

        robot.drivetrain.driveTeleOp(strafe, drive, turn);


        robot.turret.HEIGHT_OFFSET = 2 / robot.drivetrain.robotPose.getY(DistanceUnit.INCH);
        // Intake & Arms
        if (gamepad1.left_bumper) {
            trigger = false;
            robot.intake.setPower(-1);
            robot.linkage.ON();
        } else if (gamepad1.right_bumper) {
            robot.linkage.OFF();
            if (!trigger) {
                timer.reset();
                trigger = true;
            }
            robot.intake.setPower(Range.clip(-1 + (timer.seconds() * kTime), -1, -0.76));

        } else if (gamepad1.a) {
            trigger = false;
            robot.linkage.OFF();
            robot.intake.setPower(1);
        } else {
            trigger = false;
            robot.intake.setPower(0);
            robot.linkage.ON();
        }

        /*
        if (gamepad1.left_bumper) {
            robot.intake.setPower(-1);
        } else if (gamepad1.a) {
            robot.intake.setPower(1);
            robot.linkage.OFF();
        }

        if (gamepad1.right_bumper) {
            robot.intake.setPower(-1);
            robot.linkage.OFF();
        } else if (!gamepad1.a && !gamepad1.left_bumper){
            robot.intake.stop();
            robot.linkage.ON();
        }


         */

        if (gamepad1.dpad_up) {
            robot.shooter.VEL_ADJUST += 4;
        } else if (gamepad1.dpad_down) {
            robot.shooter.VEL_ADJUST -= 4;
        }



        if (gamepad1.dpad_right) {
            robot.turret.ANGLE_ADJUST -= 1;
        } else if (gamepad1.dpad_left) {
            robot.turret.ANGLE_ADJUST += 1;
        }
        /*

        if (gamepad1.y) {
            tempHoodAngle += 0.01;
        } else if (gamepad1.x) {
            tempHoodAngle -= 0.01;
        }
        robot.shooter.setHoodAngle(tempHoodAngle);


         */
        /*
        if (gamepad1.right_trigger > 0.1) {
            robot.intake.kV += 0.0001;
        } else if (gamepad1.left_trigger > 0.1) {
            robot.intake.kV -= 0.0001;
        }


         */


        /*
        if (gamepad1.left_trigger > 0.1) {
            robot.update(2); // TRACKING WHEN PRESSED
        } else {
            robot.update(0); // IDLE WHEN NOT PRESSED
        }

         */
        robot.update(2);
        /*
        robot.shooter.setTargetVelocity(targetVel);

         */

        if (gamepad1.b) {
            robot.drivetrain.setPosition(-63, -60, 0);
        }

        // Distance to goal
        double goalX = robot.targetGoal.getX(DistanceUnit.INCH);
        double goalY = robot.targetGoal.getY(DistanceUnit.INCH);

        double robotX = robot.drivetrain.robotPose.getX(DistanceUnit.INCH);
        double robotY = robot.drivetrain.robotPose.getY(DistanceUnit.INCH);

        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double distanceToGoal = Math.hypot(dx, dy);

        // === TELEMETRY (NOW ALSO GOES TO DASHBOARD) ===
        telemetry.addData("Target Velocity:", robot.shooter.getTargetVelocity());
        telemetry.addData("CUR VEL MAIN:", robot.shooter.getVelocityL());
        telemetry.addData("CUR VEL ROLLERS:", robot.shooter.getVelocityR());
        telemetry.addLine();
        //  telemetry.addData("kPL:", robot.shooter.kPL);
        //     telemetry.addData("kVL:", robot.shooter.kVL);
        //    telemetry.addLine();
        //     telemetry.addData("kPR:", robot.shooter.kPR);
        //     telemetry.addData("kVR:", robot.shooter.kVR);
        //     telemetry.addLine();
        telemetry.addData("POWER MAIN:", robot.shooter.getPowerL());
        telemetry.addData("POWER ROLLERS:", robot.shooter.getPowerR());
        telemetry.addLine();
        telemetry.addData("HOOD ANGLE:", tempHoodAngle);
        telemetry.addLine();
        telemetry.addData("TARGET TURRET ANGLE:", robot.turret.getTargetAngle());
    //    telemetry.addData("Teleop Target Turret:", targetTurretAngle);
        telemetry.addData("SERVO POS:", robot.turret.getServoPos());
        //  telemetry.addData("INTAKE kP:", robot.intake.kP);
        //  telemetry.addData("INTAKE kV:", robot.intake.kV);
        //  telemetry.addData("INTAKE TARGET:", targetIntakeVel);
        //  telemetry.addData("INTAKE VELOCITY:", robot.intake.currentVel);
        telemetry.addData("INTAKE POWER:", robot.intake.getPower());
        telemetry.addData("INTAKE CURRENT:", robot.intake.current());
        telemetry.addLine();
        telemetry.addData("ROBOT X:", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
        telemetry.addData("ROBOT Y:", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
        telemetry.addData("ROBOT HEADING:", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("DIST TO GOAL (in):", distanceToGoal);
        /**
         *
         *             Y
         *        X    :     B
         *             A
         *
         */
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

        telemetry.update();
    }
}