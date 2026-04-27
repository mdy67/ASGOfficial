package org.firstinspires.ftc.teamcode.VIPER;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Linkage {

    Servo Linkage2, Linkage1;

    final double ON_POS = 0.22;
    final double OFF_POS = 0.11;

    final double OFFSET = 0.0084;

    public Linkage(HardwareMap hardwareMap) {
        Linkage2 = hardwareMap.get(Servo.class, "linkage2");
        Linkage1 = hardwareMap.get(Servo.class, "linkage1");
    }

    public void ON() {
        Linkage2.setPosition(ON_POS - OFFSET);
        Linkage1.setPosition(ON_POS);
    }

    public void OFF() {
        Linkage2.setPosition(OFF_POS - OFFSET);
        Linkage1.setPosition(OFF_POS);
    }
}
