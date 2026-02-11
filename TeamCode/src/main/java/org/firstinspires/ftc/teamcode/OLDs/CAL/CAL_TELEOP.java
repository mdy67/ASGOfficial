package org.firstinspires.ftc.teamcode.OLDs.CAL;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.OLDs.CAL.Flywheel.CALFlywheelClass;
@Disabled
@TeleOp(name = "Nolan is amazing", group = "Testing")
public class CAL_TELEOP extends OpMode {
    private DcMotorEx flywheelMotor, intake, leftFront, leftBack, rightFront, rightBack;
    private CRServo spinner;

    private double prevTicks;

    private static final double CPR = 564;
    private double RPM, intakePWR;
    private double peakAmps;
    private double loopCounter = 0;

    private double tgtVEL = 250;
    private double loopTime;

    CALFlywheelClass flywheel;


    @Override public void init() { // INIT BUTTON PRESSED
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new CALFlywheelClass(hardwareMap, battery);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightBack");  // intentionally swapped
        rightBack = hardwareMap.get(DcMotorEx.class, "rightFront");  // intentionally swapped

        // Directions — KEEP SAME
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
     //   flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");

        spinner = hardwareMap.get(CRServo.class, "spinner");

        // Set flywheel motor directions
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      //  flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    //    flywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    }

    public void move(double drive, double strafe, double rotate, boolean slowMode) {
        double speedMultiplier = slowMode ? 0.3 : 1.0;

        double lf = drive + strafe + rotate;
        double lb = drive - strafe + rotate;
        double rf = drive + strafe - rotate;
        double rb = drive - strafe - rotate;

        // Normalize to prevent ratio distortion
        double max = Math.max(1.0, Math.abs(lf));
        max = Math.max(max, Math.abs(lb));
        max = Math.max(max, Math.abs(rf));
        max = Math.max(max, Math.abs(rb));

        lf /= max;
        lb /= max;
        rf /= max;
        rb /= max;

        leftFront.setPower(lf * speedMultiplier);
        leftBack.setPower(lb * speedMultiplier);
        rightFront.setPower(rf * speedMultiplier);
        rightBack.setPower(rb * speedMultiplier);
    }

    @Override
    public void loop() { // PLAY BUTTON PRESSED, CODE RUNNING
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        boolean slowMode = gamepad1.right_trigger > 0.1;
        move(drive, strafe, rotate, slowMode);

        if (Math.abs(gamepad2.right_stick_y) > 0.01f) {
            tgtVEL += gamepad2.right_stick_y * -4;
        }

        if (gamepad2.left_trigger > 0.1f) {
            intakePWR = -1;
        } else if (gamepad2.left_bumper){
            intakePWR = 1;
        } else if (!gamepad2.a){
            intakePWR = 0;
        } else {
            intakePWR = 1; // goof
        }

        intake.setPower(intakePWR);

        if (gamepad2.a) {
            spinner.setPower(-1);
            intake.setPower(intakePWR);
        } else {
            spinner.setPower(0);
        }

        flywheel.setTargetVelocity(tgtVEL);
        flywheel.update();

        telemetry.addData("TARGET VELOCITY:", flywheel.getTargetVelocity());
        telemetry.addLine();
        telemetry.addData("SHOOTER VELOCITY:", flywheel.getVelocityRadPerSec());
        telemetry.addData("SHOOTER RPM:", flywheel.getVelocityRPM());
        telemetry.addData("SHOOTER POWER:", flywheel.getPower());
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("INTAKE PWR:", intakePWR);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("tgtVEL:", tgtVEL);
        packet.put("Current Vel:", flywheel.getVelocityRadPerSec());
        packet.put("ESTIMATED RPM:", tgtVEL * 9.549297);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);


    }
}
