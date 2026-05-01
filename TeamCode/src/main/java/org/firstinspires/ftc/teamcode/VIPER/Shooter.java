package org.firstinspires.ftc.teamcode.VIPER;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.*;

public class Shooter {

    private final DcMotorEx shooterL, shooterR;
    private final Servo hoodL, hoodR;

    private boolean useExternalControl = false;

    // ---------------- LEFT (MAIN FLYWHEEL - TUNED) ----------------

    public double kPfar = 0.036;
    public double kVfar = 0.002;
    public double kSfar = 0.052;
    public double kDfar = 0.0003;

    public double kPclose = 0.036;
    public double kVclose = 0.002;
    public double kSclose = 0.052;
    public double kDclose = 0.0003;

    private double lastErrorL = 0;

    // ---------------- RIGHT (COUNTER ROLLERS - KEEP OLD) ----------------

    public double kPRfar = 0.0049;
    public double kVRfar = 0.0023;

    // ---------------- GENERAL ----------------

    private double lastControlTime = 0;

    private static final double HOOD_TOP = 0.46;
    private static final double HOOD_BOTTOM = 0.025;

    private double targetVelocity = 0.0;

    public double MAIN_TOLERANCE = 12;

    public double velocityModifier = -53.0;
    public double MIN_VELOCITY = 210;
    public double MAX_VELOCITY = 800;
    public double VEL_ADJUST = 0;

    public static final double THRESHOLD = 10;

    // ---------------- NEW: VELOCITY LOOKUP TABLE ----------------

    private final double[] DISTANCES = {40, 45, 58.3, 72.97, 90, 109.6, 125.9, 135.3, 142, 147, 150};
    private final double[] VELOCITIES = {264, 296, 308, 324, 348, 384, 405, 429, 455, 466, 480};

    // Zone control
    public double Y_THRESHOLD = -15; // FIXED
    public double CLOSE_MIN = 250;
    public double CLOSE_MAX = 380;
    public double FAR_MIN = 390;
    public double FAR_MAX = 500;

    private boolean isClose = false; // NEW

    // ---------------- VELOCITY ----------------

    private final Deque<Double> velBufferL = new ArrayDeque<>();
    private final Deque<Double> velBufferR = new ArrayDeque<>();
    private final int BUFFER_SIZE = 5;

    private double smoothedVelocityL = 0.0;
    private double smoothedVelocityR = 0.0;

    private double lastTicksL = 0.0;
    private double lastTicksR = 0.0;
    private double lastVelocityTime = 0.0;

    public boolean shooting = false;
    public double kTime = 0.5;
    public double maxPower = -0.55;

    boolean stoppage = false;

    public Shooter(HardwareMap hardwareMap) {

        hoodL = hardwareMap.get(Servo.class, "hoodL");
        hoodR = hardwareMap.get(Servo.class, "hoodR");

        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastTicksL = -shooterL.getCurrentPosition();
        lastTicksR = -shooterR.getCurrentPosition();

        double now = System.nanoTime() / 1e9;
        lastVelocityTime = now;
        lastControlTime = now;
    }

    // ---------------- TUNER CONTROL ----------------

    public void setRawPowerL(double power) {
        useExternalControl = true;
        shooterL.setPower(-power);
    }

    public boolean atVelocity() {
        return (getVelocityL() - targetVelocity) <= MAIN_TOLERANCE;
    }

    public void setRawPower(double power) {
        useExternalControl = true;
        shooterL.setPower(-power);
        shooterR.setPower(-power);
    }

    // ---------------- UPDATE ----------------

    public void update() {

        updateVelocity();

        if (useExternalControl) return;

        double now = System.nanoTime() / 1e9;
        double dt = now - lastControlTime;
        lastControlTime = now;

        double errorL = targetVelocity - smoothedVelocityL;
        double errorR = targetVelocity - smoothedVelocityR;

        // SELECT CONSTANTS BASED ON ZONE
        double kP = isClose ? kPclose : kPfar;
        double kV = isClose ? kVclose : kVfar;
        double kS = isClose ? kSclose : kSfar;
        double kD_use = isClose ? kDclose : kDfar;

        // LEFT
        double ffL = kS + (kV * targetVelocity);
        double dL = (dt > 0) ? (errorL - lastErrorL) / dt : 0;

        double powerL = ffL + (kP * errorL) + (kD_use * dL);
        lastErrorL = errorL;

        // RIGHT (unchanged)
        double powerR = (kVRfar * targetVelocity) + (kPRfar * errorR);

        powerL = Range.clip(powerL, -0.2, 1.0);
        powerR = Range.clip(powerR, 0.0, 1.0);

        if (!stoppage) {
            shooterL.setPower(-powerL);
            shooterR.setPower(-powerR);
        }
    }

    // ---------------- VELOCITY ----------------

    public void updateVelocity() {

        double now = System.nanoTime() / 1e9;
        double dt = now - lastVelocityTime;

        double ticksL = -shooterL.getCurrentPosition();
        double ticksR = -shooterR.getCurrentPosition();

        double velL = 0, velR = 0;

        if (dt > 0) {
            velL = (ticksL - lastTicksL) / dt * 2 * Math.PI / 28.0;
            velR = (ticksR - lastTicksR) / dt * 2 * Math.PI / 28.0;
        }

        velBufferL.addLast(velL);
        velBufferR.addLast(velR);

        if (velBufferL.size() > BUFFER_SIZE) velBufferL.removeFirst();
        if (velBufferR.size() > BUFFER_SIZE) velBufferR.removeFirst();

        smoothedVelocityL = velBufferL.stream().mapToDouble(d -> d).average().orElse(0);
        smoothedVelocityR = velBufferR.stream().mapToDouble(d -> d).average().orElse(0);

        lastTicksL = ticksL;
        lastTicksR = ticksR;
        lastVelocityTime = now;
    }

    // ---------------- GETTERS ----------------

    public double getVelocityL() { return smoothedVelocityL; }
    public double getVelocityR() { return smoothedVelocityR; }

    public double getVelocityRadPerSec() {
        return (smoothedVelocityL + smoothedVelocityR) / 2.0;
    }

    public double getPowerL() { return shooterL.getPower(); }
    public double getPowerR() { return shooterR.getPower(); }

    public double getTargetVelocity() { return targetVelocity; }

    // ---------------- TARGET ----------------

    public void setTargetVelocity(double v) {
        targetVelocity = v + VEL_ADJUST;
        useExternalControl = false;
        stoppage = false;
    }

    // ---------------- STOP ----------------

    public void stop() {
        stoppage = true;
        shooterL.setPower(0);
        shooterR.setPower(0);
    }

    // ---------------- HOOD ----------------

    public void setHoodAngle(double pos) {
        pos = Range.clip(pos, HOOD_BOTTOM, HOOD_TOP);
        hoodL.setPosition(pos);
        hoodR.setPosition(pos);
    }

    public double getHoodAngle() {
        return hoodL.getPosition();
    }

    public void angleHood(double v) {
        double[] xs = VELOCITIES; // acc distances
        double[] ys = {0.025, 0.21, 0.31, 0.38, 0.435, 0.435, 0.408, 0.389, 0.379, 0.377, 0.394};

        double target = ys[0];
        for (int i = 0; i < xs.length - 1; i++) {
            if (v >= xs[i] && v <= xs[i+1]) {
                double t = (v - xs[i])/(xs[i+1]-xs[i]);
                target = ys[i] + t*(ys[i+1]-ys[i]);
                break;
            }
        }
        setHoodAngle(target);
    }

    // ---------------- NEW: INTERPOLATION ----------------

    private double interpolateVelocity(double d) { // 466 at 147

        if (d <= DISTANCES[0]) return VELOCITIES[0];
        if (d >= DISTANCES[DISTANCES.length - 1]) return VELOCITIES[VELOCITIES.length - 1];

        for (int i = 0; i < DISTANCES.length - 1; i++) {
            if (d >= DISTANCES[i] && d <= DISTANCES[i + 1]) {

                double t = (d - DISTANCES[i]) / (DISTANCES[i + 1] - DISTANCES[i]);
                return VELOCITIES[i] + t * (VELOCITIES[i + 1] - VELOCITIES[i]);
            }
        }

        return VELOCITIES[0];
    }

    // ---------------- AIM ----------------

    public void aimToGoal(Pose2D goal, Pose2D pose) {

        double dx = goal.getX(DistanceUnit.INCH) - pose.getX(DistanceUnit.INCH);
        double dy = goal.getY(DistanceUnit.INCH) - pose.getY(DistanceUnit.INCH);

        double d = Math.sqrt(dx * dx + dy * dy);

        double v = interpolateVelocity(d);

        double robotY = pose.getY(DistanceUnit.INCH);

        // FIXED LOGIC
        isClose = robotY > Y_THRESHOLD;

        if (isClose) {
            v = Range.clip(v, CLOSE_MIN, CLOSE_MAX) - velocityModifier * 0.7; // TODO: goon on the niggas in our div
        } else {
            v = Range.clip(v, FAR_MIN, FAR_MAX);
        }

        v += velocityModifier;

        setTargetVelocity(v);
        angleHood(v);
    }
}