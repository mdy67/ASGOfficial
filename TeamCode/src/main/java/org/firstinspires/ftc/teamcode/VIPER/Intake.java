package org.firstinspires.ftc.teamcode.VIPER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayDeque;
import java.util.Deque;

public class Intake {

    private final DcMotorEx intakeL, intakeR;

    public double kP = 0;
    public double kV = 0;
    public double targetVel = 0;
    private double lastTicks = 0;
    private double lastTime = 0;
    public double power = 0;

    public double currentVel;

    private final double BUFFER_SIZE = 10;

    private final Deque<Double> velBuffer = new ArrayDeque<>();

    public Intake(HardwareMap hardwareMap) {
        intakeL = hardwareMap.get(DcMotorEx.class, "intakeL");
        intakeR = hardwareMap.get(DcMotorEx.class, "intakeR");
        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lastTicks = intakeL.getCurrentPosition();
        lastTime = System.nanoTime() / 1e9;
    }

    public void setPower(double power) {
        intakeL.setPower(power);
        intakeR.setPower(power);
    }

    public double getPower() {
        return intakeL.getPower();
    }

    private void updateVelocity() {
        double currentTime = System.nanoTime() / 1e9;

        double currentTicks = -intakeL.getCurrentPosition();
        double deltaTime = currentTime - lastTime;

        double currentVelocity = 0;

        if (deltaTime > 0) {
            currentVelocity = (currentTicks - lastTicks) / deltaTime * 2 * Math.PI / 28;
        }

        velBuffer.addLast(currentVelocity);

        if (velBuffer.size() > BUFFER_SIZE) velBuffer.removeFirst();

        currentVel = velBuffer.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

        lastTicks = currentTicks;
        lastTime = currentTime;
    }

    public void setVelocity(double targetVelo) {
        targetVel = targetVelo;
    }

    public void update() {
        updateVelocity();

        double feedforward = (kV * targetVel);
        double error = targetVel - currentVel;

        power = feedforward + (kP * error);

        power = Range.clip(power, -1.0, 1.0);

        if (targetVel != 0){
            setPower(power);
        } else {
            stop();
        }
    }

    public double current() {
        return (intakeL.getCurrent(CurrentUnit.AMPS) + intakeR.getCurrent(CurrentUnit.AMPS)) / 2;
    }


    public void stop() {
        targetVel = 0;
        intakeL.setPower(0);
        intakeR.setPower(0);
    }

}