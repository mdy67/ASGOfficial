package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayDeque;
import java.util.Deque;

public class Flywheel {

    private final DcMotorEx shooterL, shooterR;
    private final VoltageSensor battery;

    private double targetVelocity = 0.0;
    private double kP = 0.005;
    private double kV = 0.00163;
    private double kS = 0.09765;
    private double tunedVoltage = 13.0283;

    private final Deque<Double> velBuffer = new ArrayDeque<>();
    private final int BUFFER_SIZE = 18;
    private double smoothedVelocity = 0.0;

    private double lastTicks = 0.0;
    private double lastTime = 0.0;

    public static final double THRESHOLD = 80; // Threshold Radians Per Second

    public Flywheel(HardwareMap hardwareMap, VoltageSensor battery) {
        this.battery = battery;

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

    public void setTargetVelocity(double radPerSec) { this.targetVelocity = radPerSec; }

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

        shooterL.setPower(-power);
        shooterR.setPower(-power);
    }

    public void stop() {
        shooterL.setPower(0.0);
        shooterR.setPower(0.0);
    }

    public void aimToGoal(Pose2D targetGoal, Pose2D currentPose, double velX, double velY) {
        double a = 3.92579 * Math.pow(10, -8); // TODO: TUNE THESE WITH NEW REGRESSION
        double b = 1.43101;
        double c = 216.48882;


        double tx = targetGoal.getX(DistanceUnit.INCH);
        double ty = targetGoal.getY(DistanceUnit.INCH);
        double x = currentPose.getX(DistanceUnit.INCH);
        double y = currentPose.getY(DistanceUnit.INCH);

        double kVX = 0.25; // Drivetrain velocity coefficients
        double kVY = 0.25;

        double dx = tx - (x + (kVX * velX));
        double dy = ty - (y + (kVY * velY));
        double distance = Math.sqrt(dx * dx + dy * dy);

        // ax^4 + bx^1 + c
         double targetVelABCD = ((a * (Math.pow(distance, 4))) + (b * Math.pow(distance, 1)) + c);

        setTargetVelocity(targetVelABCD);
    }
}
