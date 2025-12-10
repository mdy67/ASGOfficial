package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arms {

    public Servo arm1, arm2, arm3;

    // Saved positions — tune these with the test TeleOp
    public double arm1_off = 0.4, arm1_on = 0.2;
    public double arm2_off = 0.816, arm2_on = 0.602;
    public double arm3_off = 0.61, arm3_on = 0.4;
    public double arm3_rapid = 0.51;

    public Arms(HardwareMap hw) {
        arm1 = hw.get(Servo.class, "arm1");
        arm2 = hw.get(Servo.class, "arm2");
        arm3 = hw.get(Servo.class, "arm3");

        // Start all arms off
        arm1.setPosition(arm1_off);
        arm2.setPosition(arm2_off);
        arm3.setPosition(arm3_off);
    }

    // ===== ARM 1 =====
    public void arm1_flickON()  { arm1.setPosition(arm1_on); }
    public void arm1_flickOFF() { arm1.setPosition(arm1_off); }

    // ===== ARM 2 =====
    public void arm2_flickON()  { arm2.setPosition(arm2_on); }
    public void arm2_flickOFF() { arm2.setPosition(arm2_off); }

    // ===== ARM 3 =====
    public void arm3_flickON()     { arm3.setPosition(arm3_on); }
    public void arm3_flickOFF()    { arm3.setPosition(arm3_off); }
    public void arm3_flickRAPID()  { arm3.setPosition(arm3_rapid); }

}
