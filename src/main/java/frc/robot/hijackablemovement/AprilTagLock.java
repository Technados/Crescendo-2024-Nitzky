package frc.robot.hijackablemovement;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
//import frc.utils.LimelightHelpers;

public class AprilTagLock implements MovementSource {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public static PIDController rotationPID = createPIDController();

    private static PIDController createPIDController() {
        PIDController pid = new PIDController(.01, .01, .001); // original: .01, .02, .001
        pid.setTolerance(1); // allowable angle error
        pid.enableContinuousInput(0.5, 360); // it is faster to go 1 degree from 359 to 0 instead of 359 degrees
        pid.setSetpoint(0); // 0 = apriltag angle
        return pid;
    }

    @Override
    public double getR() {
        return limelight_aim_proportional();
        // return rotationPID.calculate(table.getEntry("tx").getDouble(0));
    }

    @Override
    public double getXSpeed() {
        return limelight_range_proportional();
    }

    double limelight_aim_proportional() {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our
        // proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .005;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        // rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = table.getEntry("tx").getDouble(0) * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= Constants.DriveConstants.kMaxAngularSpeed / 4;

        // invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    double limelight_range_proportional() {
        double kP = 0.0095;

        double targetingForwardSpeed = table.getEntry("ty").getDouble(0) * kP;
        targetingForwardSpeed *= Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        // targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;

    }
}

// if (table.getEntry("ty").getDouble(0) > 15.5) {
// return 0;
// } else {
// double targetingForwardSpeed = table.getEntry("ty").getDouble(0) * kP;
// targetingForwardSpeed *= Constants.DriveConstants.kMaxSpeedMetersPerSecond /
// 8;
// targetingForwardSpeed *= -1.0;
// return targetingForwardSpeed;
// }