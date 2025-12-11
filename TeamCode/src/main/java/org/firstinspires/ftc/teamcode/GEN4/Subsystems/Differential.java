package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

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
    public static double kP = 0.0005;
    public static double kD = 0.001;
    public static double maxPower = 0.5;
    public static double toleranceTicks = 20;

    // --------------------
    // Physical angle / encoder mapping
    // --------------------
    public static final double MIN_ANGLE = 0;
    public static final double MAX_ANGLE = 180;
    public static double slot1Pos = -5300;
    public static double slot2Pos = -2300;
    public static double slot3Pos = 0;
    public static double angleScale = 37.78; // 3400 ticks = 90 deg → 1 deg ≈ 37.78 ticks

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

    public double compensatedAngle = 0; // target angle in degrees 0-180

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
     * Aim differential toward goal directly, ignoring velocities.
     */
    public void aimToGoal(Pose2D currentPose, Pose2D targetGoal) {
        double dx = targetGoal.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
        double dy = targetGoal.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);

        // Desired angle relative to robot heading
        double desiredAngle = Math.toDegrees(Math.atan2(dy, dx)) - currentPose.getHeading(AngleUnit.DEGREES);
        desiredAngle = Range.clip(desiredAngle, MIN_ANGLE, MAX_ANGLE);

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

        // Deadzone
        if (Math.abs(errorL) < toleranceTicks) errorL = 0;
        if (Math.abs(errorR) < toleranceTicks) errorR = 0;

        double dL = errorL - lastErrorL;
        double dR = errorR - lastErrorR;
        lastErrorL = errorL;
        lastErrorR = errorR;

        double powerL = Range.clip(errorL * kP + dL * kD, -maxPower, maxPower);
        double powerR = Range.clip(errorR * kP + dR * kD, -maxPower, maxPower);

        diffyL.setPower(powerL);
        diffyR.setPower(powerR);
    }

    // --------------------
    // Utility
    // --------------------
    public int getEncoderL() { return encL.getCurrentPosition(); }
    public int getEncoderR() { return encR.getCurrentPosition(); }
}
