package org.firstinspires.ftc.teamcode.GEN4.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Differential;

@TeleOp(name = "Differential Gantry Test", group = "TEST")
public class DifferentialGantryTest extends LinearOpMode {

    private Differential diff;

    double angle = 0;
    int slot = 3;

    boolean lbPrev = false, rbPrev = false;

    @Override
    public void runOpMode() {

        diff = new Differential(hardwareMap);
        diff.goToSlot(3);
        diff.setTargetAngle(0);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                slot = 1; angle = 0;
                diff.goToSlot(slot);
            } else if (gamepad1.b) {
                slot = 2; angle = 0;
                diff.goToSlot(slot);
            } else if (gamepad1.y) {
                slot = 3; angle = 0;
                diff.goToSlot(slot);
            }

            if (gamepad1.left_bumper && !lbPrev) angle -= 0.1;
            if (gamepad1.right_bumper && !rbPrev) angle += 0.1;

            angle = Range.clip(angle, 0, 180);
            diff.setTargetAngle(angle);

            lbPrev = gamepad1.left_bumper;
            rbPrev = gamepad1.right_bumper;

            diff.update();

            telemetry.addLine("=== DIFFY DEBUG ===");
            telemetry.addData("Slot", slot);
            telemetry.addData("Angle", angle);
          //  telemetry.addData("Slot Base", diff.slot3Pos);
            telemetry.addData("Target L / R", "%.1f / %.1f", diff.targetL, diff.targetR);
         //   telemetry.addData("Error L / R", "%.1f / %.1f", diff.errorL, diff.errorR);
            telemetry.addData("Enc L / R", "%.1f / %.1f", diff.getEncoderL(), diff.getEncoderR());
            telemetry.addData("At Target", diff.atTarget);
            telemetry.update();
        }
    }
}
