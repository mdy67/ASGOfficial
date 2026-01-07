package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp (name = "LIMELIGHT TEST", group = "GEN4")
public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;
    Robot robot;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);

        robot = new Robot(hardwareMap);
        robot.startup();
        robot.drivetrain.state = Drivetrain.State.TELEOP;

        robot.arms.arm1_flickOFF();
        robot.arms.arm2_flickOFF();
        robot.arms.arm3_flickOFF();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        robot.update();
        robot.readMotifTag();
        telemetry.addData("TAG ID:", robot.MotifTagID);
        telemetry.update();

    }
}
