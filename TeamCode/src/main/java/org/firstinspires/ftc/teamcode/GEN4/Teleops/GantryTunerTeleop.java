package org.firstinspires.ftc.teamcode.GEN4.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Differential;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;

    @TeleOp(name = "Gantry Tuner TeleOp", group = "GEN4")
    public class GantryTunerTeleop extends OpMode {

        Differential differential;
        //   double targetVel = 200;
        @Override
        public void init() {
            differential = new Differential(hardwareMap);
            differential.resetEncoders();

        }
        private boolean TUNING_MODE = false;
        @Override
        public void loop() {
            differential.update();
            // ---------- DRIVING ----------
            if (gamepad1.dpad_right) {
                differential.goToSlot(3);
            } else if (gamepad1.dpad_down) {
                differential.goToSlot(2);
            } else if (gamepad1.dpad_left) {
                differential.goToSlot(1);
            }

            telemetry.addLine("=== DIFFERENTIAL ===");
            telemetry.addData("Encoders L/R", "%d / %d",
                    differential.encL.getCurrentPosition(),
                    differential.encR.getCurrentPosition());
            telemetry.addData("Target L/R", "%.1f / %.1f",
                    differential.targetL,
                    differential.targetR);
            telemetry.addData("Target Angle", "%.1f",
                    differential.compensatedAngle);

            telemetry.update();
        }
    }


