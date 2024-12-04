// ArmSubsystem.java
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    // Subsystem for controlling the robot's arm, used for note deflection and endgame climbing.

    // Define motor controllers for the left and right arm motors.
    private final CANSparkMax leftArmMotor = new CANSparkMax(ArmConstants.kLeftArmMotorCanId, MotorType.kBrushless);
    private final CANSparkMax rightArmMotor = new CANSparkMax(ArmConstants.kRightArmMotorCanId, MotorType.kBrushless);

    // Encoders for tracking the position of the left and right arm motors.
    private final RelativeEncoder leftEncoder = leftArmMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightArmMotor.getEncoder();

    public ArmSubsystem() {
        // Reset motors to factory defaults for consistency and safety.
        leftArmMotor.restoreFactoryDefaults();
        rightArmMotor.restoreFactoryDefaults();

        // Set inversion and idle modes for motors as defined in ArmConstants.
        leftArmMotor.setInverted(ArmConstants.kLeftArmInverted);
        rightArmMotor.setInverted(ArmConstants.kRightArmInverted);
        leftArmMotor.setIdleMode(ArmConstants.kArmIdleMode);
        rightArmMotor.setIdleMode(ArmConstants.kArmIdleMode);

        // Save motor configurations to non-volatile memory.
        leftArmMotor.burnFlash();
        rightArmMotor.burnFlash();

        // Reset encoder values to 0.0 for initial calibration.
        resetEncoders();
    }

    // Resets both arm motor encoders to a position of 0.0.
    public void resetEncoders() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }

    // Returns the position of the left arm encoder.
    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition();
    }

    // Returns the position of the right arm encoder.
    public double getRightEncoderPosition() {
        return rightEncoder.getPosition();
    }

    // Calculates and returns the average position of the left and right arm encoders.
    public double getAverageArmEncoderDistance() {
        return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
    }

    // Moves the arm based on input trigger values, stopping movement at defined limits.
    public void moveArm(double forwardTrigger, double reverseTrigger) {
        if (forwardTrigger > 0.1) {
            if (getAverageArmEncoderDistance() < ArmConstants.kArmForwardLimit) {
                leftArmMotor.set(ArmConstants.kArmSpeed);
                rightArmMotor.set(ArmConstants.kArmSpeed);
            } else {
                stopArm(); // Stop if the arm reaches the forward limit.
            }
        } else if (reverseTrigger > 0.1) {
            if (getAverageArmEncoderDistance() > ArmConstants.kArmReverseLimit) {
                leftArmMotor.set(-ArmConstants.kArmSpeed);
                rightArmMotor.set(-ArmConstants.kArmSpeed);
            } else {
                stopArm(); // Stop if the arm reaches the reverse limit.
            }
        } else {
            stopArm(); // Stop the arm if no trigger is pressed.
        }
    }

    // Spins both arm motors forward at a preset speed.
    public void armForward() {
        leftArmMotor.set(ArmConstants.kArmSpeed);
        rightArmMotor.set(ArmConstants.kArmSpeed);
    }

    // Spins both arm motors in reverse at a preset speed.
    public void armReverse() {
        leftArmMotor.set(-ArmConstants.kArmSpeed);
        rightArmMotor.set(-ArmConstants.kArmSpeed);
    }

    // Stops both arm motors.
    public void stopArm() {
        leftArmMotor.set(0.0);
        rightArmMotor.set(0.0);
    }

    // Autonomous method to extend the arm out to a predefined position.
    public boolean autoArmOut() {
        resetEncoders();
        while (getAverageArmEncoderDistance() < ArmConstants.kAutoArmForwardLimit) {
            armForward();
        }
        stopArm();
        return true; // Indicate the task is complete.
    }

    // Autonomous method to retract the arm back to its starting position.
    public boolean autoArmIn() {
        resetEncoders();
        while (getAverageArmEncoderDistance() > ArmConstants.kAutoArmReverseLimit) {
            armReverse();
        }
        stopArm();
        return true; // Indicate the task is complete.
    }
}
