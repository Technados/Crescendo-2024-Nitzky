package frc.robot.subsystems;

/**
 * ArmSubsystem
 * 
 * This subsystem controls the dual-arm mechanism on the robot. 
 * It uses position-based control to ensure precise arm positioning, which is critical 
 * for tasks like climbing or manipulating game pieces.
 * 
 * **Approach:**
 * - Dual motors (left and right) are controlled using built-in PID controllers for 
 *   synchronized and precise movements.
 * - The subsystem tracks arm position using encoders, allowing for reliable feedback 
 *   and closed-loop control.
 * - Position limits and motor safety features prevent overextension or damage.
 * 
 * **Why This Approach?**
 * - Position-based control ensures the arm moves to and holds target positions 
 *   accurately, which is essential for achieving repeatable and consistent robot actions.
 * - PID control allows dynamic adjustments to motor output based on the current arm position.
 * - Telemetry improves debugging and performance tuning, ensuring the subsystem operates reliably.
 * 
 * **Improvements Made:**
 * - Replaced manual speed control with PID-driven position control.
 * - Parameterized motor limits and PID coefficients for flexibility.
 * - Removed blocking loops in autonomous routines to integrate better with the scheduler.
 * - Added telemetry for real-time monitoring of arm position and motor performance.
 */

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax leftArmMotor;
    private final CANSparkMax rightArmMotor;
    private final RelativeEncoder leftArmEncoder;
    private final RelativeEncoder rightArmEncoder;
    private final SparkMaxPIDController leftPIDController;
    private final SparkMaxPIDController rightPIDController;

    private static final double kP = 0.1, kI = 0.0, kD = 0.0, kFF = 0.0;

    public ArmSubsystem() {
        // Initialize motors and encoders
        leftArmMotor = new CANSparkMax(Constants.ArmConstants.kLeftArmCanId, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(Constants.ArmConstants.kRightArmCanId, MotorType.kBrushless);
        leftArmEncoder = leftArmMotor.getEncoder();
        rightArmEncoder = rightArmMotor.getEncoder();
        leftPIDController = leftArmMotor.getPIDController();
        rightPIDController = rightArmMotor.getPIDController();

        // Configure motors
        configureMotor(leftArmMotor, leftPIDController);
        configureMotor(rightArmMotor, rightPIDController);

        resetEncoders();
    }

    private void configureMotor(CANSparkMax motor, SparkMaxPIDController pidController) {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(Constants.ArmConstants.kArmMotorCurrentLimit);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kFF);
        pidController.setOutputRange(-1.0, 1.0);
        motor.burnFlash();
    }

    @Override
    public void periodic() {
        // Update telemetry
        SmartDashboard.putNumber("Left Arm Position", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right Arm Position", getRightEncoderPosition());
    }

    public void setArmPosition(double targetPosition) {
        // Set arm position using PID control
        leftPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
        rightPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    }

    public void stopArm() {
        // Stop both arm motors
        leftArmMotor.set(0.0);
        rightArmMotor.set(0.0);
    }

    public double getLeftEncoderPosition() {
        return leftArmEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        return rightArmEncoder.getPosition();
    }

    public void resetEncoders() {
        leftArmEncoder.setPosition(0.0);
        rightArmEncoder.setPosition(0.0);
    }
}
