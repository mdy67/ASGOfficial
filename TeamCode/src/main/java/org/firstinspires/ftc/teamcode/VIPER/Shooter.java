package org.firstinspires.ftc.teamcode.VIPER;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayDeque;
import java.util.Deque;

public class Shooter {

    private final DcMotorEx shooterL, shooterR;
    private final Servo hoodL, hoodR;


    private static final double HOOD_TOP = 0.9;
    private static final double HOOD_BOTTOM = 0.025;

    private double targetVelocity = 0.0;
    public double kPL = 0.083;
    public double kVL = 0.002;
    public double kPR = 0.0049;
    public double kVR = 0.0023;


    private final Deque<Double> velBufferL = new ArrayDeque<>();
    private final Deque<Double> velBufferR = new ArrayDeque<>();
    private final int BUFFER_SIZE = 18;

    private double smoothedVelocityL = 0.0;
    private double smoothedVelocityR = 0.0;

    private double lastTicksL = 0.0;
    private double lastTicksR = 0.0;
    private double lastTime = 0.0;

    public double velocityModifier = 0.0;
    public double MIN_VELOCITY = 330;
    public double MAX_VELOCITY = 800;

    public static final double THRESHOLD = 15; // rad/s

    boolean stoppage = false;

    public Shooter(HardwareMap hardwareMap) {
        // L IS MAIN FLYWHEEL
        // R IS COUNTER ROLLERS


        hoodL = hardwareMap.get(Servo.class, "hoodL");
        hoodR = hardwareMap.get(Servo.class, "hoodR");

        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastTicksL = shooterL.getCurrentPosition(); // TODO: CHANGE
        lastTicksR = -shooterR.getCurrentPosition();
        lastTime = System.nanoTime() / 1e9;
    }

    // ---------------- HOOD ----------------

    public void setHoodAngle(double servoPos) {
        servoPos = Range.clip(servoPos, HOOD_BOTTOM, HOOD_TOP);
        hoodL.setPosition(servoPos);
        hoodR.setPosition(servoPos);
    }

    public double getHoodAngle() {
        return hoodL.getPosition();
    }

    // ---------------- TARGET ----------------

    public void setTargetVelocity(double radPerSec) {
        this.targetVelocity = radPerSec;
        stoppage = false;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    // ---------------- VELOCITY ----------------

    public void updateVelocity() {
        double currentTime = System.nanoTime() / 1e9;

        double currentTicksL = -shooterL.getCurrentPosition();
        double currentTicksR = -shooterR.getCurrentPosition();

        double deltaTime = currentTime - lastTime;

        double velL = 0.0;
        double velR = 0.0;

        if (deltaTime > 0) {
            velL = (currentTicksL - lastTicksL) / deltaTime * 2 * Math.PI / 28.0;
            velR = (currentTicksR - lastTicksR) / deltaTime * 2 * Math.PI / 28.0;
        }

        velBufferL.addLast(velL);
        velBufferR.addLast(velR);

        if (velBufferL.size() > BUFFER_SIZE) velBufferL.removeFirst();
        if (velBufferR.size() > BUFFER_SIZE) velBufferR.removeFirst();

        smoothedVelocityL = velBufferL.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        smoothedVelocityR = velBufferR.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

        lastTicksL = currentTicksL;
        lastTicksR = currentTicksR;
        lastTime = currentTime;
    }

    public double getVelocityL() {
        return smoothedVelocityL;
    }

    public double getVelocityR() {
        return smoothedVelocityR;
    }

    public double getVelocityRadPerSec() {
        return (smoothedVelocityL + smoothedVelocityR) / 2.0;
    }

    public double getVelocityRPM() {
        return getVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
    }

    public double getPowerL() {
        return shooterL.getPower();
    }

    public double getPowerR() {
        return shooterR.getPower();
    }

    public boolean atTargetVelocity() {
        return Math.abs(targetVelocity - smoothedVelocityL) <= THRESHOLD &&
                Math.abs(targetVelocity - smoothedVelocityR) <= THRESHOLD;
    }

    // ---------------- CONTROL LOOP ----------------

    public void update() {
        updateVelocity();

        double feedforwardL = (kVL * targetVelocity);
        double feedforwardR = (kVR * targetVelocity);

        double errorL = targetVelocity - smoothedVelocityL;
        double errorR = targetVelocity - smoothedVelocityR;

        double powerL = feedforwardL + (kPL * errorL);
        double powerR = feedforwardR + (kPR * errorR);

        powerL = Range.clip(powerL, 0, 1.0);
        powerR = Range.clip(powerR, 0, 1.0);

        if (!stoppage) {
            shooterL.setPower(-powerL);
            shooterR.setPower(-powerR);
        }
    }

    public void stop() {
        stoppage = true;
        shooterL.setPower(0.0);
        shooterR.setPower(0.0);
    }

    // ---------------- AIMING ----------------

    public void aimToGoal(Pose2D targetGoal, Pose2D currentPose) {
        stoppage = false;

        double a = 3.02129 * Math.pow(10, -8);
        double b = 2.27752;
        double c = 186.32838;

        double tx = targetGoal.getX(DistanceUnit.INCH);
        double ty = targetGoal.getY(DistanceUnit.INCH);
        double x = currentPose.getX(DistanceUnit.INCH);
        double y = currentPose.getY(DistanceUnit.INCH);

        double dx = tx - x;
        double dy = ty - y;
        double distance = Math.sqrt(dx * dx + dy * dy);

        double targetVel = (a * Math.pow(distance, 4)) + (b * distance) + c;

        targetVel += velocityModifier;
        targetVel = Range.clip(targetVel, MIN_VELOCITY, MAX_VELOCITY);

        setTargetVelocity(targetVel);
        angleHood(targetVel);
    }

    // ---------------- HOOD INTERPOLATION ----------------

    public void angleHood(double currentVelocity) {

        double[] xs = {320, 345, 370, 395, 430, 440, 500, 520, 555, 560};
        double[] ys = {0.79, 0.70, 0.57, 0.38, 0.37, 0.21, 0.17, 0.15, 0.39, 0.41};

        double targetHoodAngle;

        if (currentVelocity <= xs[0]) {
            targetHoodAngle = ys[0];
        } else if (currentVelocity >= xs[xs.length - 1]) {
            targetHoodAngle = ys[ys.length - 1];
        } else {
            targetHoodAngle = ys[0];

            for (int i = 0; i < xs.length - 1; i++) {
                if (currentVelocity >= xs[i] && currentVelocity <= xs[i + 1]) {
                    double x1 = xs[i];
                    double x2 = xs[i + 1];
                    double y1 = ys[i];
                    double y2 = ys[i + 1];

                    targetHoodAngle = y1 + (currentVelocity - x1) * (y2 - y1) / (x2 - x1);
                    break;
                }
            }
        }

        targetHoodAngle = Range.clip(targetHoodAngle, HOOD_TOP, HOOD_BOTTOM);
        setHoodAngle(targetHoodAngle);
    }
}