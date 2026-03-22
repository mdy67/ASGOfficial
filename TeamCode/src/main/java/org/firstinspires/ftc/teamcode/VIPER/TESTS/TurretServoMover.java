package org.firstinspires.ftc.teamcode.VIPER.TESTS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "LINKAGE MOVER", group = "VIPER")
public class TurretServoMover extends OpMode {

    double currentPos = 0;
    double currentOffset = 0.0084;
    Servo turret1, turret2; // turret3;

    @Override
    public void init() {
       turret1 = hardwareMap.get(Servo.class, "turret1");
       turret2 = hardwareMap.get(Servo.class, "turret2");
   //    turret3 = hardwareMap.get(Servo.class, "turret3");

    }



    @Override
    public void loop() {
        currentPos -= (gamepad1.right_stick_y * 0.05);
     //   currentOffset -= (gamepad1.left_stick_y * 0.001);
        currentPos = Range.clip(currentPos, 0.07, 0.265);
        turret1.setPosition(currentPos);
        turret2.setPosition(currentPos - currentOffset);
    //    turret3.setPosition(currentPos);

        telemetry.addData("CURRENT POS:", currentPos);
        telemetry.addData("OFFSET:", currentOffset);
        telemetry.update();
    }
}
