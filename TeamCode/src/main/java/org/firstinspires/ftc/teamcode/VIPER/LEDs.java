package org.firstinspires.ftc.teamcode.VIPER;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LEDs {
    Servo LED1, LED2, LED3;

    public LEDs(HardwareMap hardwareMap) {
        LED1 = hardwareMap.get(Servo.class, "LED1");
        LED2 = hardwareMap.get(Servo.class, "LED2");
        LED3 = hardwareMap.get(Servo.class, "LED3");
    }

    public void setOneColor(double pos) {
        LED1.setPosition(pos);
        LED2.setPosition(pos);
        LED3.setPosition(pos);
    }

    public void setIndividual(double pos1, double pos2, double pos3) {
        LED1.setPosition(pos1);
        LED2.setPosition(pos2);
        LED3.setPosition(pos3);
    }
}
