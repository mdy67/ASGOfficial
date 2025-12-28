package org.firstinspires.ftc.teamcode.GEN4.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Differential;

@TeleOp(name = "Differential Gantry Test", group = "TEST")
public class DifferentialGantryTest extends LinearOpMode {

    private Differential differential;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the differential
        differential = new Differential(hardwareMap);

        telemetry.addLine("Differential initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        int currentSlot = 3;   // Start at slot 1
        double targetAngle = 0; // Start angle
        timer.reset();

        while (opModeIsActive()) {

            // Simple controls to test slots
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

            // Simple controls to test angle adjustment
            if (gamepad1.left_bumper) {
                targetAngle -= 5; // decrease angle
            } else if (gamepad1.right_bumper) {
                targetAngle += 5; // increase angle
            }

            differential.setTargetAngle(targetAngle);

            // Run the update loop
            differential.update();

            // Telemetry for debugging
            telemetry.addLine("=== Differential Test ===");
            telemetry.addData("Target Slot", currentSlot);
            telemetry.addData("Target Angle", "%.2f", targetAngle);
            telemetry.addData("Compensated Angle", "%.2f", differential.compensatedAngle);
            telemetry.addData("Encoder L", differential.getEncoderL());
            telemetry.addData("Encoder R", differential.getEncoderR());
            telemetry.addData("At Target", differential.atTarget);
            telemetry.update();
        }
    }
}
