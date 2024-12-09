/**
 * ArmCommand
 *
 * This command moves the robot's arm to a specific position using PID control. 
 * It works with the ArmSubsystem and ensures the arm stops once it reaches the 
 * desired target position or is interrupted.
 *
 * **Key Features:**
 * - Uses the ArmSubsystem's `setArmPosition()` method to move the arm.
 * - Automatically ends when the arm is within a defined tolerance of the target position.
 * - Stops the arm if the command is interrupted.
 *
 * **Adjustable Values:**
 * - Position tolerance can be adjusted to fine-tune the precision of the arm's movement.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final double targetPosition;

    public ArmCommand(ArmSubsystem armSubsystem, double targetPosition) {
        this.armSubsystem = armSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        // Set the arm to the desired position
        armSubsystem.setArmPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // Check if the arm is within the tolerance range of the target position
        return Math.abs(armSubsystem.getLeftEncoderPosition() - targetPosition) < 1.0; // Adjust tolerance here
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            armSubsystem.stopArm(); // Stop the arm if interrupted
        }
    }
}
