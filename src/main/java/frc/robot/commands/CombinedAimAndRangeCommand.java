package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class CombinedAimAndRangeCommand extends CommandBase {

    private final LimelightSubsystem limelightSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    private boolean isAligned = false;
    private boolean isRanged = false;

    public CombinedAimAndRangeCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(limelightSubsystem, driveSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
        isAligned = false;
        isRanged = false;
        limelightSubsystem.setLEDMode(Constants.LimelightConstants.kLEDOn); // Turn on LEDs for targeting
    }

    @Override
    public void execute() {
        // Aiming: Adjust robot heading based on tx
        double tx = limelightSubsystem.getTx();
        if (Math.abs(tx) > Constants.LimelightConstants.kAimTolerance) {
            driveSubsystem.drive(0, 0, -Math.signum(tx) * Constants.LimelightConstants.kAimSpeed, false, true);
        } else {
            isAligned = true;
            driveSubsystem.drive(0, 0, 0, false, true);
        }

        // Ranging: Adjust shooter parameters based on distance
        double distance = limelightSubsystem.getDistance(); // Example: Get distance from Limelight
        double targetRPM = shooterSubsystem.calculateRPMFromDistance(distance);
        
        shooterSubsystem.setShooterVelocity(targetRPM); // Set shooter speed based on distance
        
        
    }

    @Override
    public boolean isFinished() {
        return isAligned && isRanged;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false, true); // Stop movement
        limelightSubsystem.setLEDMode(Constants.LimelightConstants.kLEDOff); // Turn off LEDs
        if (interrupted) {
            shooterSubsystem.stopShooter(); // Stop shooter if command is interrupted
        }
    }
}
