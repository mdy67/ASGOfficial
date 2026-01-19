package org.firstinspires.ftc.teamcode.GEN4.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.AutoToTeleop;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;

@TeleOp
public class DiffyServoTesticler extends OpMode {

    Servo diffyL, diffyR;
    double targetL = 0.3275;
    double targetR = 0.9577;


    double slot3L = 0.3275;
    double slot3R = 0.9577;
    double slotIncrement = 0.115;

    double slot2L = slot3L + slotIncrement;
    double slot2R = slot3R - slotIncrement;
    double slot1L = slot2L + slotIncrement;
    double slot1R = slot2R - slotIncrement;


    double slotOffsetR = 0;
    double slotOffsetL = 0;

    int currentSlot = 3;

    double targetAngle = 0;
    double angleScale = 0.00181944444444444444444444444444;

    @Override
    public void init() {
        diffyL = hardwareMap.get(Servo.class, "diffyL");
        diffyR = hardwareMap.get(Servo.class, "diffyR");
        telemetry.addLine("Robot initialized. Waiting for start...");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (currentSlot == 3) {
            slotOffsetL = slot3L;
            slotOffsetR = slot3R;
        } else if (currentSlot == 2) {
            slotOffsetL = slot2L;
            slotOffsetR = slot2R;
        } else if (currentSlot == 1) {
            slotOffsetL = slot1L;
            slotOffsetR = slot1R;
        } else {
            slotOffsetL = slot3L;
            slotOffsetR = slot3R;
        }

        if (gamepad1.dpad_left) {
            currentSlot = 1;
        } else if (gamepad1.dpad_down) {
            currentSlot = 2;
        } else if (gamepad1.dpad_right) {
            currentSlot = 3;
        }

        targetAngle += gamepad1.right_stick_x * 0.3;
        targetAngle = Range.clip(targetAngle, 0, 180);

        targetL = slotOffsetL - (targetAngle * angleScale);
        targetR = slotOffsetR - (targetAngle * angleScale);

        diffyL.setPosition(targetL);
        diffyR.setPosition(targetR);

        telemetry.addData("diffyL: ", diffyL.getPosition());
        telemetry.addData("diffyR:", diffyR.getPosition());
        telemetry.addData("targetL: ", targetL);
        telemetry.addData("targetR: ", targetR);
        telemetry.addData("Current slot:", currentSlot);
        telemetry.addData("Target Angle: ", targetAngle);
        telemetry.update();

    }

}
