package org.firstinspires.ftc.teamcode.VIPER.TESTS;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VIPER.Shooter;

@TeleOp(name = "Shooter PID Tuner (Simple)", group = "Tuning")
public class ShooterTuner extends OpMode {

    Shooter shooter;

    // ---------------- TUNABLES ----------------

    double targetVelocity = 200;

    double kP = 0.04;
    double kV = 0.002;
    double kS = 0.05;
    double kD = 0.0;

    boolean enableD = false;

    // ---------------- PARAM SELECT ----------------

    enum Param {
        TARGET,
        KP,
        KV,
        KS,
        KD
    }

    Param selected = Param.TARGET;

    // ---------------- INTERNAL ----------------

    private double lastError = 0;
    private double lastTime = 0;

    private boolean lastLeft = false;
    private boolean lastRight = false;
    private boolean lastStart = false;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lastTime = System.nanoTime() / 1e9;
    }

    @Override
    public void loop() {

        // ---------------- PARAM SELECTION ----------------

        if (gamepad1.dpad_left && !lastLeft) {
            selected = Param.values()[(selected.ordinal() - 1 + Param.values().length) % Param.values().length];
        }

        if (gamepad1.dpad_right && !lastRight) {
            selected = Param.values()[(selected.ordinal() + 1) % Param.values().length];
        }

        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;

        // ---------------- ADJUSTMENT ----------------

        if (gamepad1.dpad_up) adjust(true);
        if (gamepad1.dpad_down) adjust(false);

        // Toggle D (debounced)
        if (gamepad1.start && !lastStart) {
            enableD = !enableD;
        }
        lastStart = gamepad1.start;

        // ---------------- CONTROL LOOP ----------------

        shooter.updateVelocity();

        double velocity = shooter.getVelocityRadPerSec();

        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        double error = targetVelocity - velocity;

        double ff = kS + (kV * targetVelocity);

        double derivative = 0;
        if (dt > 0) derivative = (error - lastError) / dt;

        double power = ff + (kP * error);
        if (enableD) power += kD * derivative;

        lastError = error;

        power = Math.max(-0.2, Math.min(1.0, power));

        shooter.setRawPower(power);

        // ---------------- TELEMETRY ----------------

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("LEFT/RIGHT → Select");
        telemetry.addLine("UP/DOWN → Adjust");
        telemetry.addLine("START → Toggle D");

        telemetry.addLine("");

        telemetry.addLine("=== SELECTED ===");
        telemetry.addLine("> " + selected.name());

        telemetry.addLine("");

        telemetry.addLine("=== VALUES ===");
        telemetry.addData("Target Velocity", format(selected == Param.TARGET, targetVelocity));
        telemetry.addData("kP", format(selected == Param.KP, kP));
        telemetry.addData("kV", format(selected == Param.KV, kV));
        telemetry.addData("kS", format(selected == Param.KS, kS));
        telemetry.addData("kD", format(selected == Param.KD, kD));
        telemetry.addData("D Enabled", enableD);

        telemetry.addLine("");

        telemetry.addLine("=== STATE ===");
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);

        telemetry.update();
    }

    // ---------------- ADJUST FUNCTION ----------------

    private void adjust(boolean increase) {

        double dir = increase ? 1 : -1;

        switch (selected) {
            case TARGET:
                targetVelocity += dir * 5;        // bigger step for velocity
                break;
            case KP:
                kP += dir * 0.002;
                break;
            case KV:
                kV += dir * 0.0001;
                break;
            case KS:
                kS += dir * 0.002;
                break;
            case KD:
                kD += dir * 0.0001;
                break;
        }
    }

    // ---------------- FORMAT ----------------

    private String format(boolean selected, double value) {
        return (selected ? ">> " : "") + String.format("%.5f", value);
    }
}