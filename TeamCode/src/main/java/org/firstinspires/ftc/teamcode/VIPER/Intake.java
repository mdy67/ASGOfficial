package org.firstinspires.ftc.teamcode.VIPER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    private final DcMotorEx intakeL, intakeR;

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

    public void setPower(double power) {
        intakeL.setPower(power);
        intakeR.setPower(power);
    }

    public void stop() {
        intakeL.setPower(0);
        intakeR.setPower(0);
    }

}