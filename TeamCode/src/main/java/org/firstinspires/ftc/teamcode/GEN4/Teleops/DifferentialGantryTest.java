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

        // Initialize differential
        differential = new Differential(hardwareMap);

        telemetry.addLine("Differential initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        int currentSlot = 3; // Home
        double targetAngle = 0;
        timer.reset();

        while (opModeIsActive()) {

            // --- Slot controls ---
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

            // --- Angle controls ---
            if (gamepad1.left_bumper) targetAngle -= 5;
            if (gamepad1.right_bumper) targetAngle += 5;
            differential.setTargetAngle(targetAngle);

            // --- Update differential ---
            differential.update();

            // --- Telemetry ---
            telemetry.addLine("=== Differential Gantry Debug ===");
            telemetry.addData("Target Slot", currentSlot);
            telemetry.addData("Target Angle", "%.2f", targetAngle);
            telemetry.addData("Compensated Angle", "%.2f", differential.compensatedAngle);
            telemetry.addData("Encoder L", differential.getEncoderL());
            telemetry.addData("Encoder R", differential.getEncoderR());
            telemetry.addData("Target L / R", "%.1f / %.1f", differential.targetL, differential.targetR);
            telemetry.addData("At Target", differential.atTarget);
            telemetry.update();
        }
    }
}
