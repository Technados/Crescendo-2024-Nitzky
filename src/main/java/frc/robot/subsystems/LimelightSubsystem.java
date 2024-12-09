package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    public LimelightSubsystem() {
        // Initialize subsystem
        setLEDMode(Constants.LimelightConstants.kLEDOff); // Default LED state: off
        setPipeline(Constants.LimelightConstants.kDefaultPipeline); // Default pipeline
    }

    @Override
    public void periodic() {
        // Update telemetry
        SmartDashboard.putNumber("Limelight tx", getTx());
        SmartDashboard.putNumber("Limelight ty", getTy());
        SmartDashboard.putNumber("Limelight Distance", getDistance());
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
    }

    // Limelight data retrieval
    public double getTx() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    }

    public double getTargetArea() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
    }

    public boolean hasTarget() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) == 1.0;
    }

    // Distance calculation based on ty
    public double getDistance() {
        double ty = getTy();
        double angleToTarget = Constants.LimelightConstants.kMountingAngle + ty;
        return Constants.LimelightConstants.kTargetHeightDifference / Math.tan(Math.toRadians(angleToTarget));
    }

    // Limelight configuration
    public void setLEDMode(int mode) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
    }

    public void setPipeline(int pipeline) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }
}
