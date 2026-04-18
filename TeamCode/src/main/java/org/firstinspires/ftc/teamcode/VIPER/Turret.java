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

    /*
        CALIBRATION (REAL MEASURED):

        90°  → 0.1173 servo pos
        180° → 0.4066 servo pos

        Mapping used:
        servoPos = 0.1173 + (angle - 90) * 0.003214
    */

    public State state = State.IDLE;

    Servo turret1, turret2, turret3;
    DcMotorEx revCoder;

    private double SERVO_MIN_POS = 0;
    private double SERVO_MAX_POS = 1;

    private double REVCODER_SCALE = 3500; // encoder ticks across full servo range
    private double TOTALDEGS = 280;       // still used for encoder (ignored for output now)
    private double zeroServoPos = 0.0566; // no longer used for output

    private double currentTicks = 0;
    private final double TOLERANCE_DEGS = 10;
    private double TOLERANCE_TICKS = (REVCODER_SCALE / TOTALDEGS) * TOLERANCE_DEGS;

    private double targetAngle = 0;

    public double ANGLE_ADJUST = 0;
    public double BLUE_OFFSET = 5.8;
    public double RED_OFFSET = 0;
    public double HEIGHT_OFFSET = 0;

    private double SOTM_OFFSET = 0;
    private static double kVel = 0.1;

    private double tgtServoPos = 0;
    private double currentDegs = 0;

    public double offset = 0.0;

    public Turret(HardwareMap hardwareMap) {
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        turret3 = hardwareMap.get(Servo.class, "turret3");

        revCoder = hardwareMap.get(DcMotorEx.class, "intakeR"); // TODO: change if needed
    }

    public boolean atTarget() {
        return Math.abs(currentTicks - (tgtServoPos * REVCODER_SCALE)) <= TOLERANCE_TICKS;
    }

    public double getServoPos() {
        return turret1.getPosition();
    }

    public void update(Pose2D robotPos, Pose2D targetGoal, double TVel) {

        tgtServoPos = turret1.getPosition();
        currentTicks = revCoder.getCurrentPosition();

        // NOTE: encoder math unchanged
        currentDegs = (TOTALDEGS * (currentTicks / REVCODER_SCALE) - ((TOTALDEGS - 360) / 2));

        calcSOTMOffset(TVel);

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

    private void calcSOTMOffset(double TVel) {
        SOTM_OFFSET = kVel * -TVel;
    }

    private void setServoPos(double pos) {
        turret1.setPosition(pos - offset);
        turret2.setPosition(pos);
        turret3.setPosition(pos + offset);
    }

    // ---------------- NEW CORRECT MAPPING ----------------
    /*
        Uses real measured calibration points.

        HOW TO TUNE:
        - If high angles are off → adjust 0.003214 slightly
        - If everything shifts → adjust 0.1173 slightly
    */
    public void setAngle(double angle) {

        targetAngle = angle;

        double servoPos = 0.1173 + (angle - 90.0) * 0.003214;

        servoPos = Range.clip(servoPos, SERVO_MIN_POS, SERVO_MAX_POS);

        setServoPos(servoPos);
    }
    // ----------------------------------------------------

    private void aimToGoal(Pose2D robotPos, Pose2D targetGoal, boolean SOTM) {

        double dx = targetGoal.getX(DistanceUnit.INCH) - robotPos.getX(DistanceUnit.INCH);
        double dy = targetGoal.getY(DistanceUnit.INCH) - robotPos.getY(DistanceUnit.INCH);

        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        fieldAngle -= robotPos.getHeading(AngleUnit.DEGREES);

        // Normalize ONCE into 0–360
        fieldAngle = (fieldAngle % 360 + 360) % 360;

        if (SOTM) {
            fieldAngle += SOTM_OFFSET;
        }

        // 🔥 HARD LIMITS (no wrap allowed past this point)
        double min = 55;
        double max = 300;

        if (fieldAngle > max) {
            fieldAngle = max;
        } else if (fieldAngle < min) {
            fieldAngle = min;
        }

        // NO more normalization or wrap logic after this
        setAngle(fieldAngle + ANGLE_ADJUST + (alliance.isBlue() ? BLUE_OFFSET + HEIGHT_OFFSET : RED_OFFSET - HEIGHT_OFFSET));
    }

    // ---------------- DEBUG ----------------

    public double getCurrentAngleDeg() {
        return currentDegs;
    }

    public double getTargetAngle() {
        return targetAngle;
    }
}