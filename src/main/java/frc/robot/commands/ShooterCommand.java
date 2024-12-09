/**
 * ShooterCommand
 *
 * This command starts the shooter and ensures it reaches the desired RPM. 
 * It works with the ShooterSubsystem to maintain precise control of the shooter speed.
 *
 * **Key Features:**
 * - Uses PID control to achieve and hold a target RPM.
 * - Ends when the RPM is within a defined tolerance of the target value.
 * - Stops the shooter if the command is interrupted.
 *
 * **Adjustable Values:**
 * - RPM tolerance can be adjusted to control how precise the shooter needs to be.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final double targetRPM;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double targetRPM) {
        this.shooterSubsystem = shooterSubsystem;
        this.targetRPM = targetRPM;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        // Set the shooter to the desired velocity
        shooterSubsystem.setShooterVelocity(targetRPM);
    }

    @Override
    public boolean isFinished() {
        // Check if the shooter is within the RPM tolerance
        return Math.abs(shooterSubsystem.getAverageRPM() - targetRPM) < 50; // Adjust RPM tolerance here
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooterSubsystem.stopShooter(); // Stop the shooter if interrupted
        }
    }
}