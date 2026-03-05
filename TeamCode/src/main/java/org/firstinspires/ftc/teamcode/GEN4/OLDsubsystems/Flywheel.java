package org.firstinspires.ftc.teamcode.GEN4.OLDsubsystems;

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

public class Flywheel {

    private final DcMotorEx shooterL, shooterR;
    private final Servo hood;
    private final VoltageSensor battery;

    private static final double HOOD_TOP = 0.0;
    private static final double HOOD_BOTTOM = 0.79;

    private double targetVelocity = 0.0;
    private double kP = 0.004;
    private double kV = 0.0017;
    private double kS = 0.08765;
    private double tunedVoltage = 12.583;

    private final Deque<Double> velBuffer = new ArrayDeque<>();
    private final int BUFFER_SIZE = 18;
    private double smoothedVelocity = 0.0;

    private double lastTicks = 0.0;
    private double lastTime = 0.0;

    public double velocityModifier = 0.0;
    public double MIN_VELOCITY = 330;
    public double MAX_VELOCITY = 800;

    public static final double THRESHOLD = 15; // Threshold Radians Per Second

    public Flywheel(HardwareMap hardwareMap, VoltageSensor battery) {
        this.battery = battery;

        hood = hardwareMap.get(Servo.class, "hood");
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastTicks = -shooterR.getCurrentPosition();
        lastTime = System.nanoTime() / 1e9;
    }

    public void setFeedforward(double kV, double kS, double tunedVoltage) {
        this.kV = kV;
        this.kS = kS;
        this.tunedVoltage = tunedVoltage;
    }

    public void setKP(double kP) { this.kP = kP; }

    public void setHoodAngle(double servoPos) { hood.setPosition(servoPos);}
    public double getHoodAngle() { return hood.getPosition(); }

    public void setTargetVelocity(double radPerSec) { this.targetVelocity = radPerSec; stoppage = false; }

    public double getTargetVelocity() { return targetVelocity; }

    public void updateVelocity() {
        double currentTicks = -shooterR.getCurrentPosition();
        double currentTime = System.nanoTime() / 1e9;

        double deltaTicks = currentTicks - lastTicks;
        double deltaTime = currentTime - lastTime;

        double instVel = deltaTime > 0 ? (deltaTicks / deltaTime) * 2 * Math.PI / 28.0 : 0.0;

        velBuffer.addLast(instVel);
        if (velBuffer.size() > BUFFER_SIZE) velBuffer.removeFirst();

        smoothedVelocity = velBuffer.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

        lastTicks = currentTicks;
        lastTime = currentTime;
    }

    public boolean atTargetVelocity() {
        if (Math.abs(targetVelocity - getVelocityRadPerSec()) <= THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }

    public double getVelocityRadPerSec() { return smoothedVelocity; }

    public double getVelocityRPM() { return smoothedVelocity * 60.0 / (2.0 * Math.PI); }

    public double getPower() { return shooterL.getPower(); }

    public void update() {
        updateVelocity();

        double voltage = (battery != null) ? battery.getVoltage() : 13.0;
        double feedforwardPower = (kV * targetVelocity + kS) * tunedVoltage / voltage;

        double error = targetVelocity - smoothedVelocity;
        double power = feedforwardPower + (kP * error);

        power = Range.clip(power, 0.0, 1.0);

        if (!stoppage) {
            shooterL.setPower(-power);
            shooterR.setPower(-power);
        }

    }
    boolean stoppage = false;
    public void stop() {
        stoppage = true;
        shooterL.setPower(0.0);
        shooterR.setPower(0.0);
    }

    public void aimToGoal(Pose2D targetGoal, Pose2D currentPose, double velX, double velY) {
        stoppage = false;
        double a = 3.02129 * Math.pow(10, -8); // TODO: TUNE THESE WITH NEW REGRESSION
        double b = 2.27752;
        double c = 186.32838;


        double tx = targetGoal.getX(DistanceUnit.INCH);
        double ty = targetGoal.getY(DistanceUnit.INCH);
        double x = currentPose.getX(DistanceUnit.INCH);
        double y = currentPose.getY(DistanceUnit.INCH);

        double kVX = 0.2; // Drivetrain velocity coefficients
        double kVY = 0.2;

        double dx = tx - (x + (kVX * velX));
        double dy = ty - (y + (kVY * velY));
        double distance = Math.sqrt(dx * dx + dy * dy);

        // ax^4 + bx^1 + c
        double targetVelABCD = ((a * (Math.pow(distance, 4))) + (b * Math.pow(distance, 1)) + c);
        targetVelABCD += velocityModifier;
        targetVelABCD = Range.clip(targetVelABCD, MIN_VELOCITY, MAX_VELOCITY);
        setTargetVelocity(targetVelABCD);
        angleHood(targetVelABCD);
    }

    public void angleHood(double currentVelocity) {

        // Sorted velocity (x) and hood position (y) calibration points
        double[] xs = {320, 345, 370, 395, 430, 440, 500, 520, 555, 560};
        double[] ys = {0.79, 0.70, 0.57, 0.38, 0.37, 0.21, 0.17, 0.15, 0.39, 0.41};

        double targetHoodAngle;

        // Clamp outside range
        if (currentVelocity <= xs[0]) {
            targetHoodAngle = ys[0];
        } else if (currentVelocity >= xs[xs.length - 1]) {
            targetHoodAngle = ys[ys.length - 1];
        } else {
            // Find segment and linearly interpolate
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
