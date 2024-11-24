package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreSpeaker extends Command {
    private ShooterSubsystem shooterSubsystem;
    boolean stop;

    public ScoreSpeaker(ShooterSubsystem shooter) {
        this.shooterSubsystem = shooter;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stop = shooterSubsystem.autoShooter();
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
