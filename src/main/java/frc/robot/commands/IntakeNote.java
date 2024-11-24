package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNote extends Command {
    private IntakeSubsystem intakeSubsystem;
    boolean stop;

    public IntakeNote(IntakeSubsystem intake) {
        this.intakeSubsystem = intake;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stop = intakeSubsystem.autoIntake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return stop;
    }
}
