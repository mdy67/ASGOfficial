package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    private final DcMotorEx intakeL, intakeR;
    public static final double RAPID_SPEED = -1;

    public Intake(HardwareMap hardwareMap) {
        intakeL = hardwareMap.get(DcMotorEx.class, "intakeL");
        intakeR = hardwareMap.get(DcMotorEx.class, "intakeR");

        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void runIntake(double power) {
        intakeL.setPower(power);
        intakeR.setPower(power);
    }

    public void stop() {
        intakeL.setPower(0);
        intakeR.setPower(0);
    }

    public void runRapid () {
        intakeL.setPower(RAPID_SPEED);
        intakeR.setPower(RAPID_SPEED);
    }

    public void update() {
        // Nothing needed for now
    }

    // =========================
    // AMPERAGE GETTERS FOR TELEMETRY
    // =========================
    public double getCurrentL() {
        return intakeL.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
    }

    public double getCurrentR() {
        return intakeR.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
    }
}
