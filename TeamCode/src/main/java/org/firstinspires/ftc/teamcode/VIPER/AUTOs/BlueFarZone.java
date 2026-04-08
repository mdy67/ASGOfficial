package org.firstinspires.ftc.teamcode.VIPER.AUTOs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.VIPER.Drivetrain;
import org.firstinspires.ftc.teamcode.VIPER.Robot;
import org.firstinspires.ftc.teamcode.VIPER.alliance;

@Autonomous(name= "Blue Far Zone", group = "GEN4")
public class BlueFarZone extends LinearOpMode {

    private Robot robot;
    private boolean reached = false;
    private int goonCounter = 0;
    private int splineCounter = 0;

    private ElapsedTime settleTimer = new ElapsedTime();   // for shooting pose
    private ElapsedTime dtStallTimer = new ElapsedTime();  // bulletproof stall timer

    public enum State {
        SHOOTING_POSE_1,
        CORNER_INTAKES,
        THIRD_SPIKE,
        CORNER_INTAKES_2,
        OFF_LINE_POS,
    }

    private State state = State.SHOOTING_POSE_1;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        alliance.set(alliance.Color.BLUE);

        while (opModeInInit()) {
            robot.shooter.stop();
            robot.drivetrain.setPosition(-8, -63, 180);
            robot.update(1);
        }

        waitForStart();

        while (opModeIsActive()) {
            updateSequence();
            robot.update(1);
        }
    }

    private void updateSequence() {
        switch (state) {

            case SHOOTING_POSE_1:
              //  robot.intake.setPower(0);

                if (!reached) {
                    robot.intake.setPower(-0.6);
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -20, -57, AngleUnit.DEGREES, 180),
                            1, 3, 0.1);

                    if (robot.drivetrain.DTatTarget()) {
                        reached = true;
                        settleTimer.reset();
                        robot.drivetrain.state = Drivetrain.State.IDLE;
                    }
                } else {
                    if (settleTimer.seconds() >= 0.5) {  // only fire after 0.5s at target
                        robot.autoRapidFire();

                        if (robot.autoRapidFireDone()) {
                            robot.resetAutoRapidFire();
                            reached = false;
                            dtStallTimer.reset();
                            splineCounter = 0;

                            switch (goonCounter) {
                                case 0:
                                    state = State.CORNER_INTAKES;
                                    break;
                                case 1:
                                    state = State.CORNER_INTAKES_2;
                                    break;
                                case 2:
                                    state = State.CORNER_INTAKES;
                                    break;
                                case 3:
                                    state = State.CORNER_INTAKES;
                                    break;
                                case 5:
                                    state = State.CORNER_INTAKES_2;
                                    break;
                                case 4:
                                    state = State.CORNER_INTAKES_2;
                                    break;
                                case 6:
                                    state = State.CORNER_INTAKES;
                                    break;
                                case 7:
                                    state = State.OFF_LINE_POS;
                                    break;
                            }
                            goonCounter++;
                        }
                    }
                }
                break;

            case CORNER_INTAKES:
                robot.intake.setPower(-1);

                if (!reached) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -65, -62, AngleUnit.DEGREES, 180),
                            1, 3, 0.25);

                    if (robot.drivetrain.DTatTarget()) {
                        reached = true;
                        dtStallTimer.reset();
                    }
                }

                // bulletproof: move on if stuck > 1.2s
                if ((reached && dtStallTimer.seconds() >= 0.5) || dtStallTimer.seconds() > 3) {
                    reached = false;
                    splineCounter = 0;
                    state = State.SHOOTING_POSE_1;
                }
                break;

            case CORNER_INTAKES_2:
                robot.intake.setPower(-1);

                if (!reached) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -63, -28, AngleUnit.DEGREES, 190),
                            1, 3, 0.25);

                    if (robot.drivetrain.DTatTarget()) {
                        reached = true;
                        dtStallTimer.reset();
                        splineCounter = 1;
                    }
                } else if (splineCounter == 1) {
                    robot.goToPoint(new Pose2D(DistanceUnit.INCH, -60, -58, AngleUnit.DEGREES, 210),
                            1, 3, 0.25);

                    // bulletproof: consider at target if DTatTarget() or 1.2s passed
                    if (robot.drivetrain.DTatTarget() || dtStallTimer.seconds() > 4.5) {
                        reached = false;
                        splineCounter = 0;
                        state = State.SHOOTING_POSE_1;
                    }
                }
                break;

            case THIRD_SPIKE:
                state = State.SHOOTING_POSE_1;
                break;

            case OFF_LINE_POS:
                robot.intake.setPower(0);
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -40, -40, AngleUnit.DEGREES, 180),
                        1, 3, 0.25);
                break;
        }
    }
}