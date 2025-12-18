package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Differential {

    // --------------------
    // PID / control constants
    // --------------------
    public static double kP = 0.00012;
    public static double kI = 0.000013;
    public static double kD = 0.00004;
    public static double kS = 0.11;          // static offset
    public static double maxPower = 0.5;
    public static double toleranceTicks = 50;

    // Integral anti-windup limit
    public static double MAX_INTEGRAL = 5000;

    // --------------------
    // Physical angle / encoder mapping
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

    public double compensatedAngle = 0;

    // --------------------
    // Turret offset
    // --------------------
    private static final double TURRET_OFFSET_INCHES = -5.0; // turret is 5 inches backward

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
    // Targeting
    // --------------------
    public void goToSlot(int slot) {
        switch (slot) {
            case 1: currentSlotBase = slot1Pos; break;
            case 2: currentSlotBase = slot2Pos; break;
            case 3:
            default: currentSlotBase = slot3Pos; break;
        }
        updateTargets();
    }

    public void setTargetAngle(double angle) {
        angle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
        compensatedAngle = angle;
        updateTargets();
    }

    private void updateTargets() {
        targetL = currentSlotBase - (compensatedAngle * angleScale);
        targetR = -(currentSlotBase + (compensatedAngle * angleScale));
    }

    /**
     * Aim differential toward goal directly, accounting for turret offset.
     */
    public void aimToGoal(Pose2D currentPose, Pose2D targetGoal) {
        // Compute turret-adjusted position
        double headingRad = Math.toRadians(currentPose.getHeading(AngleUnit.DEGREES));
        double turretX = currentPose.getX(DistanceUnit.INCH) + TURRET_OFFSET_INCHES * Math.cos(headingRad);
        double turretY = currentPose.getY(DistanceUnit.INCH) + TURRET_OFFSET_INCHES * Math.sin(headingRad);

        double dx = targetGoal.getX(DistanceUnit.INCH) - turretX;
        double dy = targetGoal.getY(DistanceUnit.INCH) - turretY;

        double desiredAngle = Math.toDegrees(Math.atan2(dy, dx)) - currentPose.getHeading(AngleUnit.DEGREES) - 90;

        while (Math.abs(desiredAngle) > 180) {
            desiredAngle -= 180 * Math.signum(desiredAngle);
        }
        desiredAngle = Range.clip(desiredAngle, MIN_ANGLE, MAX_ANGLE);
        setTargetAngle(desiredAngle);
    }

    // --------------------
    // Update loop
    // --------------------
    public boolean atTarget;
    public void update() {
        double currL = -encL.getCurrentPosition();
        double currR = encR.getCurrentPosition();

        double errorL = targetL - currL;
        double errorR = targetR - currR;

        // Combined error tolerance check
        if (Math.abs(errorL) + Math.abs(errorR) <= toleranceTicks) {
            integralL = 0;
            integralR = 0;
        }

        // Accumulate integral
        integralL += errorL;
        integralR += errorR;

        // Anti-windup
        integralL = Range.clip(integralL, -MAX_INTEGRAL, MAX_INTEGRAL);
        integralR = Range.clip(integralR, -MAX_INTEGRAL, MAX_INTEGRAL);

        // Derivative
        double dL = errorL - lastErrorL;
        double dR = errorR - lastErrorR;

        lastErrorL = errorL;
        lastErrorR = errorR;

        // PID + static feedforward
        double powerL = (errorL * kP) + (integralL * kI) + (dL * kD);
        double powerR = (errorR * kP) + (integralR * kI) + (dR * kD);

        // Add static feedforward when moving
        if (Math.abs(errorL) > toleranceTicks) powerL += Math.signum(errorL) * kS;
        if (Math.abs(errorR) > toleranceTicks) powerR += Math.signum(errorR) * kS;

        atTarget = !(Math.abs(errorL) > toleranceTicks) && !(Math.abs(errorR) > toleranceTicks);

        // Clamp
        powerL = Range.clip(powerL, -maxPower, maxPower);
        powerR = Range.clip(powerR, -maxPower, maxPower);

        // Apply to CR servos
        diffyL.setPower(powerL);
        diffyR.setPower(powerR);
    }

    // --------------------
    // Utility
    // --------------------
    public int getEncoderL() { return encL.getCurrentPosition(); }
    public int getEncoderR() { return encR.getCurrentPosition(); }
}
