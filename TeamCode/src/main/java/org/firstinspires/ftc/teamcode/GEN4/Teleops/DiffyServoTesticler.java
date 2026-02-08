package org.firstinspires.ftc.teamcode.GEN4.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.AutoToTeleop;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Differential;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;

@TeleOp
public class DiffyServoTesticler extends OpMode {

    Servo diffyL, diffyR;
    double targetL = 0.5;
    double targetR = 0.5;


    double slot3L = 0.5;
    double slot3R = 0.5;
    double slotIncrement = 0.115;

    double slot2L = slot3L + slotIncrement;
    double slot2R = slot3R - slotIncrement;
    double slot1L = slot2L + slotIncrement;
    double slot1R = slot2R - slotIncrement;


    double slotOffsetR = 0;
    double slotOffsetL = 0;

    int currentSlot = 3;

    double targetAngle = 0.95;
    double angleScale = 0.0052777777777778;
    // 0.8, 0.1529
   // Differential diffy;

    @Override
    public void init() {
     //   diffy = new Differential(hardwareMap);
        diffyL = hardwareMap.get(Servo.class, "diffyL");
        diffyR = hardwareMap.get(Servo.class, "diffyR");
        telemetry.addLine("Robot initialized. Waiting for start...");
        telemetry.update();
    }

    @Override
    public void loop() {
        /*
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

*/

        diffyL.setPosition(targetL);
        diffyR.setPosition(targetR);



     //   diffy.setTargetAngle(targetAngle);

   //     if (gamepad1.dpad_left) {
    //        currentSlot = 1;
    //    } else if (gamepad1.dpad_down) {
     //       currentSlot = 2;
    //    } else if (gamepad1.dpad_right) {
    //        currentSlot = 3;
    //    }

     //   diffy.goToSlot(currentSlot);
    //    diffy.update();

        targetAngle += gamepad1.right_stick_x * 0.006;
        targetAngle = Range.clip(targetAngle, 0, 0.95);

        targetL = targetAngle;
        targetR = targetAngle;

      //  telemetry.addData("diffyL: ", diffyL.getPosition());
      //  telemetry.addData("diffyR:", diffyR.getPosition());
        telemetry.addData("targetL: ", targetL);
        telemetry.addData("targetR: ", targetR);
        telemetry.addData("Current slot:", currentSlot);
        telemetry.addData("Target Angle: ", targetAngle);
        telemetry.update();

    }

}
