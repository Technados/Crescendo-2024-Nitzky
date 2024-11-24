package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class ArmControlCommand extends Command {
    /** Creates a new ArmControlCommand. */
    ArmSubsystem ArmSubsystem;
    double value = 0;

    public ArmControlCommand(ArmSubsystem ArmSubsystem) {
        this.ArmSubsystem = ArmSubsystem;
        addRequirements(ArmSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting arm control command");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double leftTrigger = RobotContainer.m_operatorController.getLeftTriggerAxis()
                * -0.5;
        double rightTrigger = RobotContainer.m_operatorController.getRightTriggerAxis() * 0.5;
        value = 0;
        ArmSubsystem.moveArm(leftTrigger, rightTrigger);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
