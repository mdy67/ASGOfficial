package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Differential {

    // --------------------
    // PID / control constants (FTC DASHBOARD TUNABLE)
    // --------------------
    public static double kP = 0.00012;
    public static double kI = 0.000001;
    public static double kD = 0.0003;
    public static double kS = 0.11;
    public double ANGLE_ADJUST = 0;

    public static double maxPower = 1;
    public static double toleranceTicks = 70;
    public static double MAX_INTEGRAL = 5000;

    // --------------------
    // Physical angle limits
    // --------------------
    public static final double MIN_ANGLE = 0;
    public static final double MAX_ANGLE = 180;

    public static double slot1Pos = -5300;
    public static double slot2Pos = -2300;
    public static double slot3Pos = 0;
    public static double angleScale = 38.76;

    private double currentSlotBase = 0;

    // --------------------
    // Hardware
    // --------------------
    public CRServo diffyL, diffyR;
    public DcMotorEx encL, encR;

    // --------------------
    // State
    // --------------------
    public double targetL = 0;
    public double targetR = 0;

    private double lastErrorL = 0;
    private double lastErrorR = 0;

    private double integralL = 0;
    private double integralR = 0;

    public boolean atTarget = false;
    public double compensatedAngle = 0;

    // Turret offset
    private double TURRET_OFFSET_INCHES = -5.0;

    // --------------------
    // Constructor
    // --------------------
    public Differential(HardwareMap hardwareMap) {
        diffyL = hardwareMap.get(CRServo.class, "diffyL");
        diffyR = hardwareMap.get(CRServo.class, "diffyR");

        encL = hardwareMap.get(DcMotorEx.class, "rightFront");
        encR = hardwareMap.get(DcMotorEx.class, "intakeL");

        diffyL.setDirection(CRServo.Direction.FORWARD);
        diffyR.setDirection(CRServo.Direction.FORWARD);

        resetEncoders();
    }

    public void resetEncoders() {
        encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // --------------------
    // Slot control
    // --------------------
    public void goToSlot(int slot) {
        switch (slot) {
            case 1: currentSlotBase = slot1Pos; TURRET_OFFSET_INCHES = 5.0; break;
            case 2: currentSlotBase = slot2Pos; TURRET_OFFSET_INCHES = 0.0; break;
            case 3:
            default: currentSlotBase = slot3Pos; TURRET_OFFSET_INCHES = -5.0; break;
        }
        updateTargets();
    }

    public void setTargetAngle(double angle) {
        compensatedAngle = Range.clip(angle  + ANGLE_ADJUST, MIN_ANGLE, MAX_ANGLE);
        updateTargets();
    }

    private void updateTargets() {
        targetL = currentSlotBase - (compensatedAngle * angleScale);
        targetR = -(currentSlotBase + (compensatedAngle * angleScale));
    }

    // --------------------
    // Proper angle normalization
    // --------------------
    private double normalizeDeg(double angle) {
        angle %= 360.0;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    // --------------------
    // Aim to goal (FIXED)
    // --------------------
    public void aimToGoal(Pose2D currentPose, Pose2D targetGoal) {

        double headingRad = Math.toRadians(currentPose.getHeading(AngleUnit.DEGREES));

        double turretX = currentPose.getX(DistanceUnit.INCH)
                + TURRET_OFFSET_INCHES * Math.cos(headingRad);
        double turretY = currentPose.getY(DistanceUnit.INCH)
                + TURRET_OFFSET_INCHES * Math.sin(headingRad);

        double dx = targetGoal.getX(DistanceUnit.INCH) - turretX;
        double dy = targetGoal.getY(DistanceUnit.INCH) - turretY;

        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = currentPose.getHeading(AngleUnit.DEGREES);

        double desiredAngle = fieldAngle - robotHeading - 90.0;

        desiredAngle = normalizeDeg(desiredAngle);

        // Convert [-180,180] → [0,180] safely
        if (desiredAngle < -90) {
            desiredAngle = 180;
        } else if (desiredAngle < 0) {
            desiredAngle = 0;
        }

        setTargetAngle(desiredAngle);
    }

    // --------------------
    // Update loop
    // --------------------
    public void update() {

        double currL = -encL.getCurrentPosition();
        double currR = encR.getCurrentPosition();

        double errorL = targetL - currL;
        double errorR = targetR - currR;

        if (Math.abs(errorL) + Math.abs(errorR) <= toleranceTicks) {
            integralL = 0;
            integralR = 0;
        }

        integralL += errorL;
        integralR += errorR;

        integralL = Range.clip(integralL, -MAX_INTEGRAL, MAX_INTEGRAL);
        integralR = Range.clip(integralR, -MAX_INTEGRAL, MAX_INTEGRAL);

        double dL = errorL - lastErrorL;
        double dR = errorR - lastErrorR;

        lastErrorL = errorL;
        lastErrorR = errorR;

        double powerL = errorL * kP + integralL * kI + dL * kD;
        double powerR = errorR * kP + integralR * kI + dR * kD;

        if (Math.abs(errorL) > toleranceTicks) powerL += Math.signum(errorL) * kS;
        if (Math.abs(errorR) > toleranceTicks) powerR += Math.signum(errorR) * kS;

        atTarget = Math.abs(errorL) <= toleranceTicks && Math.abs(errorR) <= toleranceTicks && Math.abs(powerL) + Math.abs(powerR) < 0.4;

        diffyL.setPower(Range.clip(powerL, -maxPower, maxPower));
        diffyR.setPower(Range.clip(powerR, -maxPower, maxPower));
    }

    // --------------------
    // Utilities
    // --------------------
    public int getEncoderL() { return encL.getCurrentPosition(); }
    public int getEncoderR() { return encR.getCurrentPosition(); }
}
