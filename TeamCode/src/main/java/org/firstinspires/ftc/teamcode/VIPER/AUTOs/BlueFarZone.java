package org.firstinspires.ftc.teamcode.VIPER.AUTOs;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VIPER.Robot;

@Autonomous(name= "Blue Far Zone", group = "GEN4")
public class BlueFarZone {

    private Robot robot;

    public enum State {
        INITIALIZED,
        START_POSE,
        SHOOTING_POSE_1,
        FIRST_INTAKES,
        
    }



}
