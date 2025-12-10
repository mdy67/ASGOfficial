package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Differential {

    public static double kPL = 0.0006; // scaled for 3:1 overdrive
    public static double kPR = 0.0006;

    public static double maxPower = 1.0;
    public static double toleranceTicks = 50;

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

    public double appliedPowerL = 0;
    public double appliedPowerR = 0;

    public Differential(HardwareMap hardwareMap) {
        diffyL = hardwareMap.get(CRServo.class, "diffyL");
        diffyR = hardwareMap.get(CRServo.class, "diffyR");

        encL = hardwareMap.get(DcMotorEx.class, "rightFront"); // swapped hardware strings
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
        // Flip L encoder reading to match servo direction
        double flippedL = -encL.getCurrentPosition();
        double errorL = targetL - flippedL;
        double errorR = targetR - encR.getCurrentPosition();

        if (Math.abs(errorL) < toleranceTicks) errorL = 0;
        if (Math.abs(errorR) < toleranceTicks) errorR = 0;

        double powerL = Range.clip(errorL * kPL, -maxPower, maxPower);
        double powerR = Range.clip(errorR * kPR, -maxPower, maxPower);

        diffyL.setPower(powerL);
        diffyR.setPower(powerR);

        appliedPowerL = powerL;
        appliedPowerR = powerR;
    }

    public void goToSlot(int slot){
        switch(slot) {
            case 1: currentSlotBase = slot1Pos; break;
            case 2: currentSlotBase = slot2Pos; break;
            case 3:
            default: currentSlotBase = slot3Pos; break;
        }
    }

    public void setTargetAngle(double angle){
        angle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
        targetL = currentSlotBase - (angle * angleScale); // L negative target
        targetR = currentSlotBase + (angle * angleScale); // R positive target
    }

    public void aimToGoal(Pose2D currentPose, double xVelocity, double yVelocity, double tVelocity, Pose2D targetGoal){
        double dx = targetGoal.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
        double dy = targetGoal.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);
        double goalAngle = Math.toDegrees(Math.atan2(dy, dx));
        setTargetAngle(goalAngle);
    }

    public int getEncoderL() { return encL.getCurrentPosition(); }
    public int getEncoderR() { return encR.getCurrentPosition(); }
}
