package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Differential {

    public static double kPL = 0.0005;
    public static double kPR = 0.0005;
    public static double kDL = 0.000002;
    public static double kDR = 0.000002;

    public static double maxPower = 1.0;
    public static double toleranceTicks = 20;

    // Slow start settings
    public static double rampRate = 0.02;  // max power change per update

    // Angle / slot system
    public static final double MIN_ANGLE = 0;
    public static final double MAX_ANGLE = 180;
    public static double slot1Pos = -5300;
    public static double slot2Pos = -2300;
    public static double slot3Pos = 0;
    public static double angleScale = 34.4;

    private double currentSlotBase = 0;

    public CRServo diffyL, diffyR;
    public DcMotorEx encL, encR;

    public double targetL = 0;
    public double targetR = 0;

    private double lastErrorL = 0;
    private double lastErrorR = 0;

    private double rampedPowerL = 0;  // stored smoothed power
    private double rampedPowerR = 0;

    public double appliedPowerL = 0;
    public double appliedPowerR = 0;

    public double compensatedAngle = 0;

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

    public void update() {
        double currL = -encL.getCurrentPosition();
        double currR = encR.getCurrentPosition();

        double errorL = targetL - currL;
        double errorR = targetR - currR;

        if (Math.abs(errorL) < toleranceTicks) errorL = 0;
        if (Math.abs(errorR) < toleranceTicks) errorR = 0;

        double dL = errorL - lastErrorL;
        double dR = errorR - lastErrorR;
        lastErrorL = errorL;
        lastErrorR = errorR;

        double rawPowerL = Range.clip(errorL * kPL + dL * kDL, -maxPower, maxPower);
        double rawPowerR = Range.clip(errorR * kPR + dR * kDR, -maxPower, maxPower);

        // ----- Slow start ramp -----
        rampedPowerL += Range.clip(rawPowerL - rampedPowerL, -rampRate, rampRate);
        rampedPowerR += Range.clip(rawPowerR - rampedPowerR, -rampRate, rampRate);

        rampedPowerL = Range.clip(rampedPowerL, -maxPower, maxPower);
        rampedPowerR = Range.clip(rampedPowerR, -maxPower, maxPower);

        diffyL.setPower(rampedPowerL);
        diffyR.setPower(rampedPowerR);

        appliedPowerL = rampedPowerL;
        appliedPowerR = rampedPowerR;
    }

    public void goToSlot(int slot) {
        switch (slot) {
            case 1: currentSlotBase = slot1Pos; break;
            case 2: currentSlotBase = slot2Pos; break;
            case 3:
            default: currentSlotBase = slot3Pos; break;
        }
    }

    public void setTargetAngle(double angle) {
        angle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);

        // Left unchanged except for your sign flip on R
        targetL = currentSlotBase - (angle * angleScale);
        targetR = -(currentSlotBase + (angle * angleScale));
    }

    public void aimToGoal(Pose2D currentPose, double robotXVel, double robotYVel, double tVelocity, Pose2D targetGoal) {
        double dx = targetGoal.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
        double dy = targetGoal.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);

        double kVX = 0.5;
        double kVY = 0.5;
        double predictedDx = dx - robotXVel * kVX;
        double predictedDy = dy - robotYVel * kVY;

        compensatedAngle =
                Math.toDegrees(Math.atan2(predictedDy, predictedDx))
                        - currentPose.getHeading(AngleUnit.DEGREES);

        setTargetAngle(compensatedAngle);
    }

    public int getEncoderL() { return encL.getCurrentPosition(); }
    public int getEncoderR() { return encR.getCurrentPosition(); }
}
