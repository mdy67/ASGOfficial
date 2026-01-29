package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
    public static double kD = 0.00005;
    public static double kS = 0.13;

    public static double maxPower = 0.8;
    public static double toleranceTicksClose = 90;
    public static double toleranceTicksFar = 60;

    public static double slot1Pos = -6500;   // gantry slots
    public static double slot2Pos = -3000;
    public static double slot3Pos = 0;       // default starting slot
    public double slotOffset = 0;
    public static double angleScale = 0.00181944444444444444444444444444; // turret rotation scaling

    public double encLOffset;
    public double encROffset;

    public double currL = 0;
    public double currR = 0;

    public boolean farZone = true;           // for tuning tolerances
    public double currentSlotBase = 3;      // gantry position base

    // --------------------
    // Hardware
    // --------------------
    public Servo diffyL, diffyR;
    public DcMotorEx encL, encR;

    // --------------------
    // State
    // --------------------
    public double targetL = 0.3275;
    public double targetR = 0.9577;

    private double lastErrorL = 0;
    private double lastErrorR = 0;

    public double desiredAngle = 0;
    public boolean atTarget = false;
    public double compensatedAngle = 0; // turret angle offset only
    public double ANGLE_ADJUST = 0;

    private double TURRET_OFFSET_INCHES = -4.8;

  //  double slot3L = 0.3275;
  //  double slot3R = 0.9577;
    double slot3L = 0.3275;
    double slot3R = 0.9577;
    double slotIncrement = 0.115;

    double slot2L = slot3L + slotIncrement;
    double slot2R = slot3R - slotIncrement;
    double slot1L = slot2L + slotIncrement;
    double slot1R = slot2R - slotIncrement;


    double slotOffsetR = slot3R;
    double slotOffsetL = slot3L;

    double MIN_ANGLE = 0;
    double MAX_ANGLE = 180;

    // --------------------
    // Constructor
    // --------------------
    public Differential(HardwareMap hardwareMap) {
        diffyL = hardwareMap.get(Servo.class, "diffyL");
        diffyR = hardwareMap.get(Servo.class, "diffyR");

      //  encL = hardwareMap.get(DcMotorEx.class, "rightFront");
     //   encL.setDirection(DcMotorEx.Direction.REVERSE);
    //    encR = hardwareMap.get(DcMotorEx.class, "intakeL");



      //  resetToSlot3();
    }
/*
    public void resetToSlot3() {
        currentSlotBase = slot3Pos;
        compensatedAngle = 0;
        targetL = 0;
        targetR = 0;
        lastErrorL = 0;
        lastErrorR = 0;
        resetEncoders();
        diffyL.setPower(0);
        diffyR.setPower(0);
    }

 */

    public void resetEncoders() {
        encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // --------------------
    // Gantry Slot Control
    // --------------------
    public void goToSlot(int slot) {
        switch (slot) {
            case 1: currentSlotBase = 1;
                    TURRET_OFFSET_INCHES = 4.8;
                    slotOffsetL = slot1L;
                    slotOffsetR = slot1R;
                    MIN_ANGLE = 50;
                    MAX_ANGLE = 130;
                    break;
            case 2: currentSlotBase = 2;
                    TURRET_OFFSET_INCHES = 0;
                    slotOffsetL = slot2L;
                    slotOffsetR = slot2R;
                    MIN_ANGLE = 0;
                    MAX_ANGLE = 180;
                    break;
            case 3:
            default: currentSlotBase = 3;
                    TURRET_OFFSET_INCHES = -4.8;
                    slotOffsetL = slot3L;
                    slotOffsetR = slot3R;
                    MIN_ANGLE = 0;
                    MAX_ANGLE = 180;
                    break;
        }
        updateTargets();
    }

    // --------------------
    // Turret Rotation Control
    // --------------------
    public void setTargetAngle(double angle) {
        lastErrorL = 0;
        lastErrorR = 0;

        compensatedAngle = angle; // COMPENSATED ANGLE IS NEGATIVE
        updateTargets();
    }

    private void updateTargets() {
     //   targetL = -(currentSlotBase - (compensatedAngle * angleScale));
    //    targetR = currentSlotBase + (compensatedAngle * angleScale);
        compensatedAngle = Range.clip(compensatedAngle, MIN_ANGLE, MAX_ANGLE);
        targetL = slotOffsetL - (compensatedAngle * angleScale);
        targetR = slotOffsetR - (compensatedAngle * angleScale);

        targetL = Math.max(targetL, 0);
        targetR = Math.min(targetR, 0.9577);
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
    // Aim to goal
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

        desiredAngle = fieldAngle - robotHeading - 90.0;
        desiredAngle = normalizeDeg(desiredAngle);
        desiredAngle += ANGLE_ADJUST;


        if (desiredAngle < -90) desiredAngle = 180;
        else if (desiredAngle < 0) desiredAngle = 0;

        setTargetAngle(desiredAngle);
    }

    // --------------------
    // Update loop (PID without integral)
    // --------------------
    public void update() {
     //   currL = -encL.getCurrentPosition() + encLOffset;
    //    currR = encR.getCurrentPosition() + encROffset;

    //    double errorL = targetL - currL;
    //    double errorR = targetR - currR;

    //    double tolerance = farZone ? toleranceTicksFar : toleranceTicksClose;

    //    double dL = errorL - lastErrorL;
    //    double dR = errorR - lastErrorR;

     //   lastErrorL = errorL;
    //    lastErrorR = errorR;

    //    double powerL = errorL * kP + dL * kD;
    //    double powerR = errorR * kP + dR * kD;

        // Feedforward for large errors
     //   if (Math.abs(errorL) > tolerance) powerL += Math.signum(errorL) * kS;
    //    if (Math.abs(errorR) > tolerance) powerR += Math.signum(errorR) * kS;

     //   atTarget = Math.abs(errorL) <= tolerance && Math.abs(errorR) <= tolerance;

        // Apply power to servos
    //    diffyL.setPower(Range.clip(-powerL, -maxPower, maxPower));
    //    diffyR.setPower(Range.clip(powerR, -maxPower, maxPower));
        atTarget = true; // TODO: TEMPORARY, MAYBE A TIMER?! what the fuck chat
        diffyL.setPosition(targetL);
        diffyR.setPosition(targetR);
    }

    // --------------------
    // Utilities
    // --------------------
    public int getEncoderL() { return encL.getCurrentPosition(); }
    public int getEncoderR() { return encR.getCurrentPosition(); }
}
