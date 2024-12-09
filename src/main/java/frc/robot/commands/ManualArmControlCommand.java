package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmControlCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final double joystickInput;

    public ManualArmControlCommand(ArmSubsystem armSubsystem, double joystickInput) {
        this.armSubsystem = armSubsystem;
        this.joystickInput = joystickInput;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        // Pass joystick input to moveArmManually
        armSubsystem.moveArmManually(joystickInput);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopArm(); // Stop the arm when the command ends
    }

    @Override
    public boolean isFinished() {
        return false; // Manual control should run continuously until interrupted
    }
}
