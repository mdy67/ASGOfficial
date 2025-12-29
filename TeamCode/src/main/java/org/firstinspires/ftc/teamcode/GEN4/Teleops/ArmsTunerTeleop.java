package org.firstinspires.ftc.teamcode.GEN4.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arm3_SimpleServoControl")
public class ArmsTunerTeleop extends OpMode {

    private Servo arm1;
    private double position = 0.5;   // start centered

    //

    @Override
    public void init() {
        arm1 = hardwareMap.get(Servo.class, "arm1");

        position = 0.5;
        arm1.setPosition(position);

        telemetry.addLine("Arm3 Simple Servo Control Ready");
    }

    @Override
    public void loop() {
        // Right stick vertical controls servo
        // Up = decrease (toward 0), Down = increase (toward 1)
        double stick = -gamepad1.right_stick_y;

        // Scale stick to small, safe increments
        position += stick * 0.01;

        // Clamp to servo range
        position = Math.max(0.0, Math.min(1.0, position));

        // Apply to servo
        arm1.setPosition(position);

        telemetry.addData("Arm3 Position", position);
        telemetry.addData("Joystick", stick);
    }
}
