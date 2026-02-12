package org.firstinspires.ftc.teamcode.GEN4.Teleops.Tunings;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Arms_Tuner_MultiServo")
public class ArmsTunerTeleop extends OpMode {

    private Servo arm1, arm2, arm3;
    private Servo activeArm;

    private double arm1Pos = 0.5;
    private double arm2Pos = 0.5;
    private double arm3Pos = 0.5;

    private int activeArmIndex = 1;

    @Override
    public void init() {
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");

        arm1.setPosition(arm1Pos);
        arm2.setPosition(arm2Pos);
        arm3.setPosition(arm3Pos);

        activeArm = arm1;

        telemetry.addLine("Arms Tuner Ready");
        telemetry.addLine("A = arm1 | B = arm2 | X = arm3");
    }

    @Override
    public void loop() {

        // --- Select active arm ---
        if (gamepad1.a) {
            activeArm = arm1;
            activeArmIndex = 1;
        } else if (gamepad1.b) {
            activeArm = arm2;
            activeArmIndex = 2;
        } else if (gamepad1.x) {
            activeArm = arm3;
            activeArmIndex = 3;
        }

        // --- Read joystick ---
        double stick = -gamepad1.right_stick_y * 0.01;

        // --- Adjust correct arm position ---
        switch (activeArmIndex) {
            case 1:
                arm1Pos = clamp(arm1Pos + stick);
                arm1.setPosition(arm1Pos);
                break;
            case 2:
                arm2Pos = clamp(arm2Pos + stick);
                arm2.setPosition(arm2Pos);
                break;
            case 3:
                arm3Pos = clamp(arm3Pos + stick);
                arm3.setPosition(arm3Pos);
                break;
        }

        telemetry.addData("Active Arm", "arm" + activeArmIndex);
        telemetry.addData("arm1 pos", arm1Pos);
        telemetry.addData("arm2 pos", arm2Pos);
        telemetry.addData("arm3 pos", arm3Pos);
    }

    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
