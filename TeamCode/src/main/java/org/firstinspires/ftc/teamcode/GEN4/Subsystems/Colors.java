package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayDeque;
import java.util.Deque;

public class Colors {

    private ColorSensor color1, color2, color3;
    private Servo led1, led2, led3;

    private final int averageWindowMs = 250;
    private final int loopPeriodMs = 40;
    private final int bufferLength = averageWindowMs / loopPeriodMs;

    // Buffers for sensors
    private final Deque<Integer> rBuf1 = new ArrayDeque<>();
    private final Deque<Integer> gBuf1 = new ArrayDeque<>();
    private final Deque<Integer> bBuf1 = new ArrayDeque<>();
    private double sumR1 = 0, sumG1 = 0, sumB1 = 0;

    private final Deque<Integer> rBuf2 = new ArrayDeque<>();
    private final Deque<Integer> gBuf2 = new ArrayDeque<>();
    private final Deque<Integer> bBuf2 = new ArrayDeque<>();
    private double sumR2 = 0, sumG2 = 0, sumB2 = 0;

    private final Deque<Integer> rBuf3 = new ArrayDeque<>();
    private final Deque<Integer> gBuf3 = new ArrayDeque<>();
    private final Deque<Integer> bBuf3 = new ArrayDeque<>();
    private double sumR3 = 0, sumG3 = 0, sumB3 = 0;

    public enum BallState { EMPTY, GREEN, PURPLE, RED }
    private BallState state1 = BallState.EMPTY;
    private BallState state2 = BallState.EMPTY;
    private BallState state3 = BallState.EMPTY;

    public int numBalls = 0;

    public Colors(HardwareMap hardwareMap) {
        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        color3 = hardwareMap.get(ColorSensor.class, "color3");

        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");
        led3 = hardwareMap.get(Servo.class, "led3");
    }

    private void addToBuffer(Deque<Integer> buffer, int value, SumHolder sum) {
        if (buffer.size() >= bufferLength) {
            int removed = buffer.pollFirst();
            sum.value -= removed;
        }
        buffer.addLast(value);
        sum.value += value;
    }

    private static class SumHolder { double value; }

    private void addToBuffers1(int r, int g, int b) {
        addToBuffer(rBuf1, r, new SumHolder(){ {value=sumR1;} }); sumR1 = rBuf1.stream().mapToInt(Integer::intValue).sum();
        addToBuffer(gBuf1, g, new SumHolder(){ {value=sumG1;} }); sumG1 = gBuf1.stream().mapToInt(Integer::intValue).sum();
        addToBuffer(bBuf1, b, new SumHolder(){ {value=sumB1;} }); sumB1 = bBuf1.stream().mapToInt(Integer::intValue).sum();
    }

    private void addToBuffers2(int r, int g, int b) {
        addToBuffer(rBuf2, r, new SumHolder(){ {value=sumR2;} }); sumR2 = rBuf2.stream().mapToInt(Integer::intValue).sum();
        addToBuffer(gBuf2, g, new SumHolder(){ {value=sumG2;} }); sumG2 = gBuf2.stream().mapToInt(Integer::intValue).sum();
        addToBuffer(bBuf2, b, new SumHolder(){ {value=sumB2;} }); sumB2 = bBuf2.stream().mapToInt(Integer::intValue).sum();
    }

    private void addToBuffers3(int r, int g, int b) {
        addToBuffer(rBuf3, r, new SumHolder(){ {value=sumR3;} }); sumR3 = rBuf3.stream().mapToInt(Integer::intValue).sum();
        addToBuffer(gBuf3, g, new SumHolder(){ {value=sumG3;} }); sumG3 = gBuf3.stream().mapToInt(Integer::intValue).sum();
        addToBuffer(bBuf3, b, new SumHolder(){ {value=sumB3;} }); sumB3 = bBuf3.stream().mapToInt(Integer::intValue).sum();
    }

    private double avg(Deque<Integer> buffer, double sum) {
        return buffer.isEmpty() ? 0 : sum / buffer.size();
    }

    private BallState classifySensor(double r, double g, double b, int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                if (r > 25 || g > 25 || b > 25) return (g > r * 2) ? BallState.GREEN : BallState.PURPLE;
                break;
            case 2:
            case 3:
                if (r > 200 || g > 200 || b > 200) return (g > r * 3) ? BallState.GREEN : BallState.PURPLE;
                break;
        }
        return BallState.EMPTY;
    }

    private void applyLED(Servo led, BallState state) {
        switch (state) {
            case EMPTY: led.setPosition(0); break;
            case GREEN: led.setPosition(0.47); break;
            case PURPLE: led.setPosition(0.722); break;
            case RED: led.setPosition(0.277); break;
        }
    }

    public boolean hasBall;
    public void update() {
        addToBuffers1(color1.red(), color1.green(), color1.blue());
        addToBuffers2(color2.red(), color2.green(), color2.blue());
        addToBuffers3(color3.red(), color3.green(), color3.blue());

        double r1 = avg(rBuf1, sumR1), g1 = avg(gBuf1, sumG1), b1 = avg(bBuf1, sumB1);
        double r2 = avg(rBuf2, sumR2), g2 = avg(gBuf2, sumG2), b2 = avg(bBuf2, sumB2);
        double r3 = avg(rBuf3, sumR3), g3 = avg(gBuf3, sumG3), b3 = avg(bBuf3, sumB3);

        state1 = classifySensor(r1, g1, b1, 1);
        state2 = classifySensor(r2, g2, b2, 2);
        state3 = classifySensor(r3, g3, b3, 3);

        applyLED(led1, state1);
        applyLED(led2, state2);
        applyLED(led3, state3);

        getNumBalls();
        if (state1 != BallState.EMPTY || state2 != BallState.EMPTY || state3 != BallState.EMPTY) {
            hasBall = true;
        } else {
            hasBall = false;
        }
    }

    public void updateTESTING(boolean DTAtTarget, boolean FLYWHEELatTarget, boolean DIFFYatTarget) {
        if (DTAtTarget) { applyLED(led3, BallState.GREEN); } else { applyLED(led3, BallState.RED); }
        if (FLYWHEELatTarget) { applyLED(led2, BallState.GREEN); } else { applyLED(led2, BallState.RED); }
        if (DIFFYatTarget) { applyLED(led1, BallState.GREEN); } else { applyLED(led1, BallState.RED); }
    }

    public void getNumBalls() {
        numBalls = 0;
        if (state1 != BallState.EMPTY) { numBalls ++; }
        if (state2 != BallState.EMPTY) { numBalls ++; }
        if (state3 != BallState.EMPTY) { numBalls ++; }
    }

    // =========================
    // GETTERS
    // =========================
    public BallState getBall1() { return state1; }
    public BallState getBall2() { return state2; }
    public BallState getBall3() { return state3; }

    public double avgRed1()   { return avg(rBuf1, sumR1); }
    public double avgGreen1() { return avg(gBuf1, sumG1); }
    public double avgBlue1()  { return avg(bBuf1, sumB1); }

    public double avgRed2()   { return avg(rBuf2, sumR2); }
    public double avgGreen2() { return avg(gBuf2, sumG2); }
    public double avgBlue2()  { return avg(bBuf2, sumB2); }

    public double avgRed3()   { return avg(rBuf3, sumR3); }
    public double avgGreen3() { return avg(gBuf3, sumG3); }
    public double avgBlue3()  { return avg(bBuf3, sumB3); }


}
