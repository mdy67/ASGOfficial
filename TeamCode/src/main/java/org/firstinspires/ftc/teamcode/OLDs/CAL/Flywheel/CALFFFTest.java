package org.firstinspires.ftc.teamcode.OLDs.CAL.Flywheel;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems.Flywheels;

@Disabled
@TeleOp(name = "Flywheel Feedforward + kP Test", group = "Flywheel Testing")
public class CALFFFTest extends LinearOpMode {

    private Flywheels flywheel;

    // Example target velocity (rad/s)
    private double targetVelocityRad = 250;

    // Feedforward values (from tuner)
    private double kV = 0.00163;
    private double kS = 0.09765;
    private double tunedVoltage = 13.0283;

    // Proportional gain (example)
    private double kP = 0.0018;

    @Override
    public void runOpMode() {
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new Flywheels(hardwareMap, battery);

        // Set feedforward and kP
        flywheel.setFeedforward(kV, kS, tunedVoltage);
        flywheel.setKP(kP);
        flywheel.setTargetVelocity(targetVelocityRad);

        telemetry.addLine("Flywheel Feedforward + kP Test Ready");
        telemetry.addLine("Use gamepad right stick Y to adjust target velocity");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Adjust target velocity with right stick
            targetVelocityRad += -gamepad1.right_stick_y * 5; // rad/s increment
            targetVelocityRad = Math.max(targetVelocityRad, 0); // no negative

            flywheel.setTargetVelocity(targetVelocityRad);
            flywheel.update();

            telemetry.addData("Target Velocity (rad/s)", targetVelocityRad);
            telemetry.addData("Actual Velocity (rad/s)", flywheel.getVelocityRadPerSec());
            telemetry.addData("Actual Velocity (RPM)", flywheel.getVelocityRPM());
            telemetry.addData("Applied Motor Power", flywheel.getPower());
            telemetry.update();
        }

        flywheel.stop();
    }
}
