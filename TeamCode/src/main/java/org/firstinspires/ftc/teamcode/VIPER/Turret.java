package org.firstinspires.ftc.teamcode.VIPER;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Turret {
    public enum State {
        FIXED,
        TRACKING,
        SOTM,
        IDLE
    }
    public State state = State.IDLE;

    Servo turret1, turret2, turret3;
    DcMotorEx revCoder;

    private double SERVO_MIN_POS = 0;
    private double SERVO_MAX_POS = 1;


    private double REVCODER_SCALE = 3500; // TODO: servo pos * scale = current ticks
    private double TOTALDEGS = 400; // SERVO_MAX_POS = servoScale in degrees; angle / servoScale) + zeroServoPos = real degrees
    private double zeroServoPos = 0.0566;
    private double currentTicks = 0;
    private final double TOLERANCE_DEGS = 5;
    private double TOLERANCE_TICKS = (REVCODER_SCALE / TOTALDEGS) * TOLERANCE_DEGS;
    private double targetAngle = 0;
    private double tgtServoPos = 0;
    private double currentDegs = 0;


    public Turret(HardwareMap hardwareMap) {
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        turret3 = hardwareMap.get(Servo.class, "turret3");
        revCoder = hardwareMap.get(DcMotorEx.class, "intakeR"); // TODO: CHANGE IF NEEDED
    }

    public boolean atTarget() {
        return Math.abs(currentTicks - (tgtServoPos * REVCODER_SCALE)) <= TOLERANCE_TICKS;
    }

    public void update(Pose2D robotPos, Pose2D targetGoal) {
        tgtServoPos = turret1.getPosition();
        currentTicks = revCoder.getCurrentPosition();
        currentDegs = (TOTALDEGS * (currentTicks / REVCODER_SCALE) - ((TOTALDEGS - 360) / 2));

        switch (state) {
            case FIXED:
                setAngle(180);
                break;

            case TRACKING:
                aimToGoal(robotPos, targetGoal, false);
                break;

            case SOTM:
                aimToGoal(robotPos, targetGoal, true);
                break;

            case IDLE:
                break;
        }
    }

    private void setServoPos(double pos) {
        turret1.setPosition(pos);
        turret2.setPosition(pos);
        turret3.setPosition(pos);
    }

    private void setAngle(double angle) {
        setServoPos((angle / TOTALDEGS) + zeroServoPos);
    }

    private void aimToGoal(Pose2D robotPos, Pose2D targetGoal, boolean SOTM) {

        double dx = targetGoal.getX(DistanceUnit.INCH) - robotPos.getX(DistanceUnit.INCH);
        double dy = targetGoal.getY(DistanceUnit.INCH) - robotPos.getY(DistanceUnit.INCH);
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        fieldAngle = fieldAngle - robotPos.getHeading(AngleUnit.DEGREES);



        double diff = fieldAngle - currentDegs;

        if (diff > 180) {
            fieldAngle -= 360;
        } else if (diff < -180) {
            fieldAngle += 360;
        }

        fieldAngle = Range.clip(fieldAngle, -20, 380);

        setAngle(fieldAngle);
    }




}
