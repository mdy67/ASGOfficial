package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {

    public HardwareMap hardwareMap;

    public Drivetrain drivetrain;
    public Differential differential;
    public Flywheel flywheel;

    public Pose2D blueGoal = new Pose2D(DistanceUnit.INCH, -60, 60, AngleUnit.DEGREES, 0);
    public Pose2D redGoal  = new Pose2D(DistanceUnit.INCH, 60, 60, AngleUnit.DEGREES, 0);

    private boolean flywheelEnabled = false;
    public Wait wait = new Wait();  // Wait helper

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);
        differential = new Differential(hardwareMap);

        try {
            flywheel = new Flywheel(hardwareMap, hardwareMap.voltageSensor.iterator().hasNext()
                    ? hardwareMap.voltageSensor.iterator().next() : null);
        } catch (Exception e) {
            flywheel = null;
            flywheelEnabled = false;
        }
    }

    /** Reset sensors and encoders */
    public void startup() {
        drivetrain.pinpoint.resetPosAndIMU();
        differential.resetEncoders();
    }

    /** Update drivetrain, wait timer, and optionally flywheel/differential */
    public void update() {
        drivetrain.update();
        wait.update(); // Update wait every loop

        if (flywheelEnabled && flywheel != null) {
            flywheel.update();
            differential.update();
        }
    }

    /** Drive to a point using drivetrain */
    public void goToPoint(Pose2D targetPoint, double maxPower, double xyThreshold, double hThreshold){
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, xyThreshold, hThreshold);
    }

    /** Hold at a position */
    public void holdPoint(Pose2D targetPoint, double maxPower){
        drivetrain.state = Drivetrain.State.HOLD_POINT;
        drivetrain.goToPoint(targetPoint, maxPower, 0, 0);
    }

    /** Disable flywheel if present */
    public void disableFlywheel() {
        flywheelEnabled = false;
        flywheel = null;
    }
}
