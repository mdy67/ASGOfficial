package org.firstinspires.ftc.teamcode.GEN4.Teleops.Tunings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;

@TeleOp(name = "Gantry Tuner TeleOp", group = "GEN4")
public class GantryTunerTeleop extends OpMode {

    Robot robot;

    // Edge-detect variables
    private boolean lastA = false;
    private boolean shootingActive = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.startup();
    //    robot.differential.resetToSlot3();
    }

    @Override
    public void loop() {

        robot.update();

        /* =========================
           A BUTTON → START SHOOT
           ========================= */
        boolean aPressed = gamepad1.a;

        // Rising edge detect
        if (aPressed && !lastA) {
            shootingActive = true;
        }
        lastA = aPressed;

        // Run shooter state machine every loop once started
        if (shootingActive) {
            boolean finished = robot.shoot_133(true);
            if (finished) {
                shootingActive = false; // ready for next press
            }
        } else {
            // keep calling with false so it stays idle
            robot.shoot_133(false);
        }

        /* =========================
           TELEMETRY
           ========================= */
        telemetry.addLine("=== DIFFERENTIAL ===");
        telemetry.addData("Encoders L/R", "%d / %d",
                robot.differential.encL.getCurrentPosition(),
                robot.differential.encR.getCurrentPosition());
        telemetry.addData("Target L/R", "%.1f / %.1f",
                robot.differential.targetL,
                robot.differential.targetR);
        telemetry.addData("Target Angle", "%.1f",
                robot.differential.compensatedAngle);
        telemetry.addData("AT TARGET", robot.differential.atTarget);
        telemetry.addData("Shooting Active", shootingActive);
        telemetry.update();
    }
}
