/**
 * IntakeCommand
 *
 * This command runs the intake to collect game pieces. It uses the IntakeSubsystem and
 * automatically ends when a game piece is detected or the command is interrupted.
 *
 * **Key Features:**
 * - Stops the intake motor when a game piece is detected via the IR sensor.
 * - Allows adjustable motor speed for different intake rates.
 *
 * **Adjustable Values:**
 * - Motor speed can be passed as a parameter when creating the command.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final double targetCurrent;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double targetCurrent) {
        this.intakeSubsystem = intakeSubsystem;
        this.targetCurrent = targetCurrent;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        // Run the intake motor at the specified current
        intakeSubsystem.runIntakeAtCurrent(targetCurrent);
    }

    @Override
    public boolean isFinished() {
        // Stop when a game piece is detected
        return intakeSubsystem.isGamePieceDetected();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake(); // Stop the intake motor
    }
}
