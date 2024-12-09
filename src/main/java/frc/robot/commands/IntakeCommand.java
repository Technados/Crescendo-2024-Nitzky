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
    private final double speed;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        // Start the intake motor
        intakeSubsystem.runIntake(speed);
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
