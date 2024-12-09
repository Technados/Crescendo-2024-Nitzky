/**
 * LimelightCommand
 *
 * This command aligns the robot to a target using the Limelight camera's `tx` (horizontal offset).
 * It uses the DriveSubsystem to rotate the robot until the target is centered.
 *
 * **Key Features:**
 * - Automatically ends when the robot is aligned within a defined tolerance.
 * - Turns the Limelight LEDs on during alignment and off when the command ends.
 * - Uses field-relative control for smooth alignment.
 *
 * **Adjustable Values:**
 * - Alignment tolerance (`kAimTolerance`) and rotation speed (`kAimSpeed`) in Constants.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightCommand extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;
    private final DriveSubsystem driveSubsystem;

    public LimelightCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.driveSubsystem = driveSubsystem;
        addRequirements(limelightSubsystem, driveSubsystem);
    }

    @Override
    public void initialize() {
        // Turn on Limelight LEDs
        limelightSubsystem.setLEDMode(Constants.LimelightConstants.kLEDOn);
    }

    @Override
    public void execute() {
        // Align the robot using the Limelight's tx value
        double tx = limelightSubsystem.getTx();
        if (Math.abs(tx) > Constants.LimelightConstants.kAimTolerance) {
            driveSubsystem.drive(0, 0, -Math.signum(tx) * Constants.LimelightConstants.kAimSpeed, true); // Adjust speed here
        } else {
            driveSubsystem.drive(0, 0, 0, true); // Stop rotation when aligned
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Turn off Limelight LEDs and stop robot movement
        limelightSubsystem.setLEDMode(Constants.LimelightConstants.kLEDOff);
        driveSubsystem.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        // Check if the robot is aligned
        return Math.abs(limelightSubsystem.getTx()) < Constants.LimelightConstants.kAimTolerance; // Adjust tolerance here
    }
}
