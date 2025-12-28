package org.firstinspires.ftc.teamcode.GEN4.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Differential;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;

@TeleOp(name = "Differential Gantry Diagnostic", group = "TEST")
public class DifferentialGantryDiagnostic extends LinearOpMode {

    private Differential differential;
    private ElapsedTime timer = new ElapsedTime();
    private Drivetrain drivetrain;
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize differential
        differential = new Differential(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        telemetry.addLine("Differential initialized. Waiting for start...");
        telemetry.update();
        drivetrain.state = Drivetrain.State.IDLE;
        waitForStart();
        int currentSlot = 3;      // Start at slot 3
        double targetAngle = 0;   // Start angle
        timer.reset();

        while (opModeIsActive()) {

            // -------------------------
            // Slot switching
            // -------------------------
            if (gamepad1.dpad_up) {
                currentSlot = 1;
                differential.goToSlot(currentSlot);
                timer.reset();
            } else if (gamepad1.dpad_left) {
                currentSlot = 2;
                differential.goToSlot(currentSlot);
                timer.reset();
            } else if (gamepad1.dpad_down) {
                currentSlot = 3;
                differential.goToSlot(currentSlot);
                timer.reset();
            }
            // -------------------------
            // Angle adjustment
            // -------------------------
            if (gamepad1.left_bumper) {
                targetAngle -= 0.5; // small step
                differential.setTargetAngle(targetAngle);
            } else if (gamepad1.right_bumper) {
                targetAngle += 0.5;
                differential.setTargetAngle(targetAngle);
            }

            // -------------------------
            // Run update
            // -------------------------
            differential.update();

            // -------------------------
            // Telemetry for debugging
            // -------------------------
            telemetry.addLine("=== Differential Diagnostic ===");
            telemetry.addData("Current Slot", currentSlot);
            telemetry.addData("Target Angle", "%.2f", targetAngle);
            telemetry.addData("Compensated Angle", "%.2f", differential.compensatedAngle);
            telemetry.addData("Encoder L/R", "%d / %d",
                    differential.getEncoderL(),
                    differential.getEncoderR());
            telemetry.addData("Target L/R", "%.2f / %.2f",
                    differential.targetL,
                    differential.targetR);
            telemetry.addData("At Target", differential.atTarget);
            telemetry.update();
        }
    }

}
