package org.firstinspires.ftc.teamcode.VIPER;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GEN4.Misc.DTPID;
import org.firstinspires.ftc.teamcode.GEN4.Misc.Utils;

public class Drivetrain {

    public enum State {
        GO_TO_POINT,
        TELEOP,
        HOLD_POINT,
        IDLE
    }
    public State state = State.IDLE;

    public DcMotor leftFront, leftBack, rightBack, rightFront;
    GoBildaPinpointDriver pinpoint;

    // -------------------------
    //     TELEOP INPUT VARS
    // -------------------------
    private double teleopStrafe = 0;
    private double teleopForward = 0;
    private double teleopTurn = 0;


    public Drivetrain(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");



        pinpoint.setOffsets(69.19, -154.75, DistanceUnit.MM); // TODO: TUNE



        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }


    // -------------------------
    //  POSE / MOVEMENT TOOLS
    // -------------------------
    public void setPosition(double x, double y, double heading) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading));
    }

    private void setMotorPowers(double lf, double lb, double rb, double rf) {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
        rightFront.setPower(rf);
    }

    public void setWeightedMotorPowers(double strafe, double fwd, double heading) {
        double denominator = Math.max(Math.abs(strafe) + Math.abs(fwd) + Math.abs(heading), 1);
        double[] weightPowers = new double[]{
                (fwd - strafe - heading) / denominator,
                (fwd + strafe - heading) / denominator,
                (fwd - strafe + heading) / denominator,
                (fwd + strafe + heading) / denominator
        };
        setMotorPowers(weightPowers[0], weightPowers[1], weightPowers[2], weightPowers[3]);
    }

    // -------------------------
    // TELEOP CONTROL METHOD
    // -------------------------
    public void driveTeleOp(double strafe, double forward, double turn) {
        teleopStrafe = strafe;
        teleopForward = forward;
        teleopTurn = turn;
    }


    // -------------------------
    // GOTOPOINT VARIABLES
    // -------------------------
    public Pose2D robotPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public Pose2D lastrobotPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 0.001, 0.001, AngleUnit.DEGREES, 0.001);
    private Pose2D lastTarget = targetPose;

    public double targetX, targetY, targetT, maxPower, xyThreshold, hThreshold;

    public double XVel() { return pinpoint.getVelX(DistanceUnit.INCH); }
    public double YVel() { return pinpoint.getVelY(DistanceUnit.INCH); }
    public double TVel() { return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS); }

    public void goToPoint(Pose2D targetPoint, double maxPower, double xyThreshold, double hThreshold) {
        if (targetPoint != lastTarget) {
            xShutoff = false;
            yShutoff = false;
            tShutoff = false;
            targetPose = targetPoint;
            lastTarget = targetPose;

            targetX = targetPose.getX(DistanceUnit.INCH);
            targetY = targetPose.getY(DistanceUnit.INCH);
            targetT = Math.toRadians(targetPose.getHeading(AngleUnit.DEGREES));

            this.maxPower = maxPower;

            this.xyThreshold = xyThreshold;
            this.hThreshold = hThreshold;
        }
    }


    // -------------------------
    // PID CONTROL FOR AUTON
    // -------------------------
    public double tError, xError, yError;
    public double globalXerror, globalYerror;

    public boolean atX, atY, atT;

    public void getErrors() {
        double heading = robotPose.getHeading(AngleUnit.RADIANS);
        tError = Utils.headingClip(targetT - heading);
        globalXerror = (targetX - robotPose.getX(DistanceUnit.INCH));
        globalYerror = (targetY - robotPose.getY(DistanceUnit.INCH));

        xError = globalXerror * Math.cos(heading) + globalYerror * Math.sin(heading);
        yError = globalYerror * Math.cos(heading) - globalXerror * Math.sin(heading);

        atX = (Math.abs(xError) <= xyThreshold / 2);
        atY = (Math.abs(xError) <= xyThreshold / 2);
        atT = (Math.abs(tError) <= hThreshold);
    }

    public boolean DTatTarget() {
        return ((Math.abs(xError) + Math.abs(yError)) <= xyThreshold && Math.abs(tError) < hThreshold);
    }

    public static final double xkP = 0.08;
    public static final double xkD = 0.01;
    public static final double ykP = 0.07;
    public static final double ykD = 0.01;
    public static final double tkP = 0.14;
    public static final double tkD = 0.03;

    DTPID xPID = new DTPID(xkP,xkD);
    DTPID yPID = new DTPID(ykP, ykD);
    DTPID tPID = new DTPID(tkP, tkD);

    public double xPower, yPower, tPower;
    private boolean xShutoff = false;
    private boolean yShutoff = false;
    private boolean tShutoff = false;

    private double xONmult = 1.5;
    private double yONmult = 1.5;
    private double tONmult = 1.5;

    public void applyPIDPowers() {
        getErrors();

        xPower = xPID.newPDPower(xError, maxPower);
        yPower = yPID.newPDPower(yError, maxPower);
        tPower = tPID.newPDPower(tError, maxPower);

        //   if (xShutoff && Math.abs(xError) > xyThreshold * xONmult) { xShutoff = false; }
        //  if (yShutoff && Math.abs(yError) > xyThreshold * yONmult) { yShutoff = false; }
        //  if (tShutoff && Math.abs(tError) > hThreshold * tONmult) { tShutoff = false; }

        //  if (atX) { xShutoff = true; xPower = 0; }
        //  if (atY) { yShutoff = true; yPower = 0; }
        //   if (atT) { tShutoff = true; tPower = 0; }

        if (atX) { xPower *= 0.8; }
        if (atY) { yPower *= 0.8; }
        if (atT) { tPower *= 0.8; }

        setWeightedMotorPowers(yPower, xPower, tPower);
    }


    // -------------------------
    //       MAIN UPDATE
    // -------------------------
    public void update(){
        pinpoint.update();
        lastrobotPose = robotPose;
        robotPose = pinpoint.getPosition();

        switch (state) {

            case GO_TO_POINT:
                applyPIDPowers();
                if (DTatTarget()) state = State.IDLE;
                break;

            case TELEOP:
                // *** FIXED — TeleOp now works ***
                setWeightedMotorPowers(teleopStrafe, teleopForward, teleopTurn);
                break;

            case HOLD_POINT:
                applyPIDPowers();
                break;

            case IDLE:
                setMotorPowers(0,0,0,0);
                break;
        }
    }

}

