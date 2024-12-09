package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
            driveSubsystem.drive(0, 0, -Math.signum(tx) * Constants.LimelightConstants.kAimSpeed, true);
        } else {
            isAligned = true;
            driveSubsystem.drive(0, 0, 0, true);
        }

        // Ranging: Adjust shooter parameters based on distance
        if (limelightSubsystem.hasTarget() && isAligned) {
            double distance = limelightSubsystem.getDistance();
            double targetRPM = ShooterSubsystem.calculateRPMFromDistance(distance); // Implement this method in ShooterSubsystem
            shooterSubsystem.setShooterVelocity(targetRPM);
            isRanged = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isAligned && isRanged;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true); // Stop movement
        limelightSubsystem.setLEDMode(Constants.LimelightConstants.kLEDOff); // Turn off LEDs
        if (interrupted) {
            shooterSubsystem.stopShooter(); // Stop shooter if command is interrupted
        }
    }
}
