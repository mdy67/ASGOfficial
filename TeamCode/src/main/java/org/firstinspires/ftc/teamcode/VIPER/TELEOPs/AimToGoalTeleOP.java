package org.firstinspires.ftc.teamcode.VIPER.TELEOPs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GEN4.OLDsubsystems.AutoToTeleop;
import org.firstinspires.ftc.teamcode.OLDs.CONFIG.RobotConfig;
import org.firstinspires.ftc.teamcode.VIPER.Robot;
import org.firstinspires.ftc.teamcode.VIPER.alliance;

@Config
@TeleOp(name = "AimToGoalTeleOP", group = "GEN4")
public class AimToGoalTeleOP extends OpMode {

    private Robot robot;

    double targetVel = 0;
    double targetIntakeVel = 0;
    double tempHoodAngle = 0;
    private double kTime;
    private double minReduc;

    private double maxReduc = -0.78;
    boolean trigger = false;

    public static double headingkP = 0.01;
    ElapsedTime timer;

    double targetTurretAngle = 180;

    @Override

    public void init() {
        timer = new ElapsedTime();
        robot = new Robot(hardwareMap);
        kTime = robot.shooter.kTime;
        minReduc = robot.shooter.maxPower;

        telemetry.addLine("Robot initialized. Waiting for start...");
        telemetry.update();
        robot.drivetrain.setPosition(AutoToTeleop.storedPose.getX(DistanceUnit.INCH), AutoToTeleop.storedPose.getY(DistanceUnit.INCH), AutoToTeleop.storedPose.getHeading(AngleUnit.DEGREES));

        // TODO: CHANGE
        //    alliance.set(alliance.Color.BLUE);

    }

    double counter = 0;

    @Override
    public void init_loop() {
        if (alliance.isBlue()) {
            robot.turret.ANGLE_ADJUST = 3.8; // TODO: TUNE FOR AFTER AUTO POS
        } else {
            robot.turret.ANGLE_ADJUST = -3; // TODO: TUNE FOR AFTER AUTO POS
        }

        if (counter < 100) {
            robot.drivetrain.setPosition(AutoToTeleop.storedPose.getX(DistanceUnit.INCH), AutoToTeleop.storedPose.getY(DistanceUnit.INCH), AutoToTeleop.storedPose.getHeading(AngleUnit.DEGREES));
        }

        robot.update(0); // 0 = IDLE

        telemetry.addLine("IN INIT");
        telemetry.addLine();
        telemetry.addData("ROBOT X:", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
        telemetry.addData("ROBOT Y:", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
        telemetry.addData("ROBOT HEADING:", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("savedPose X:", AutoToTeleop.storedPose.getX(DistanceUnit.INCH));
        telemetry.addData("savedPose Y:", AutoToTeleop.storedPose.getY(DistanceUnit.INCH));
        telemetry.addData("savedPose H:", AutoToTeleop.storedPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("COUNTER:", counter);
        counter ++;
        telemetry.update();

    }


    @Override
    public void loop() {


        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
/*
        if (gamepad1.right_trigger > 0.01) {
            drive *= 0.4;
            strafe *= 0.4;
            turn *= 0.4;
        }

 */

        double goalX = robot.targetGoal.getX(DistanceUnit.INCH);
        double goalY = robot.targetGoal.getY(DistanceUnit.INCH);

        double robotX = robot.drivetrain.robotPose.getX(DistanceUnit.INCH);
        double robotY = robot.drivetrain.robotPose.getY(DistanceUnit.INCH);

        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        if (!(gamepad1.right_trigger > 0.1)) {
            robot.drivetrain.driveTeleOp(strafe, drive, turn);
        } else {
            robot.drivetrain.driveTeleOp(strafe, drive, (fieldAngle - robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES))*headingkP);
        }


        if (gamepad1.left_bumper) {
            trigger = false;
            robot.intake.setPower(-1);
            robot.linkage.ON();
            if (robot.intake.current() >= 5.9) {
                gamepad1.rumble(80);
            }
        } else if (gamepad1.right_bumper) {
            robot.linkage.OFF();
            if (!trigger) {
                timer.reset();
                trigger = true;
            }
            robot.shooter.shooting = true;
            robot.intake.setPower(Range.clip(maxReduc + (timer.seconds() * kTime), maxReduc, minReduc));

        } else if (gamepad1.a) {
            trigger = false;
            robot.linkage.OFF();
            robot.intake.setPower(1);
        } else {
            trigger = false;
            robot.intake.setPower(0);
            robot.linkage.ON();
            robot.shooter.shooting = false;
        }

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


        robot.update(0);


        if (gamepad1.b) {
            if (alliance.isBlue()) {
                robot.drivetrain.setX(63);
            } else {
                robot.drivetrain.setX(-63);
            }

        }

        if (gamepad1.x) {
            robot.drivetrain.setY(-60);
        }

        if (gamepad1.y) {
            if (alliance.isBlue()) {
                robot.drivetrain.setHeading(180);
                robot.drivetrain.setHeading(180);
                robot.drivetrain.setHeading(180);
                robot.drivetrain.setHeading(180);
                robot.drivetrain.setHeading(180);
                robot.drivetrain.setHeading(180);
                robot.drivetrain.setHeading(180);
                robot.drivetrain.setHeading(180);

            } else {
                robot.drivetrain.setHeading(0);
                robot.drivetrain.setHeading(0);
                robot.drivetrain.setHeading(0);
                robot.drivetrain.setHeading(0);
                robot.drivetrain.setHeading(0);
                robot.drivetrain.setHeading(0);
                robot.drivetrain.setHeading(0);

            }
        }
        /*

                          TOP - HEADING (INTAKE AWAY)
                LEFT - Y POS (BOTTOM WALL)        RIGHT - X POS (CLOSE WALL)
                            BOTTOM - OUTTAKE


         */





        double distanceToGoal = Math.hypot(dx, dy);

        // === TELEMETRY (NOW ALSO GOES TO DASHBOARD) ===
        telemetry.addData("Target Velocity:", robot.shooter.getTargetVelocity());
        telemetry.addData("CUR VEL MAIN:", robot.shooter.getVelocityL());
        telemetry.addData("CUR VEL ROLLERS:", robot.shooter.getVelocityR());
        telemetry.addData("VEL ADJUST:", robot.shooter.VEL_ADJUST);
        telemetry.addData("ANGLE ADJUST:", robot.turret.ANGLE_ADJUST);
        telemetry.addLine();

        telemetry.addData("POWER MAIN:", robot.shooter.getPowerL());
        telemetry.addData("POWER ROLLERS:", robot.shooter.getPowerR());
        telemetry.addLine();
        telemetry.addData("HOOD ANGLE:", tempHoodAngle);
        telemetry.addLine();
        telemetry.addData("TARGET TURRET ANGLE:", robot.turret.getTargetAngle());
        telemetry.addData("SERVO POS:", robot.turret.getServoPos());
        telemetry.addData("INTAKE POWER:", robot.intake.getPower());
        telemetry.addData("INTAKE CURRENT:", robot.intake.current());
        telemetry.addLine();
        telemetry.addData("ROBOT X:", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
        telemetry.addData("ROBOT Y:", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
        telemetry.addData("ROBOT HEADING:", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("DIST TO GOAL (in):", distanceToGoal);

        telemetry.addLine();

        telemetry.update();
    }
}