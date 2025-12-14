package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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
        limelight.updateRobotOrientation(robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPoseFromTag = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
        }
    }
}
