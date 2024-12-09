package frc.robot.subsystems;

/**
 * IntakeSubsystem
 * 
 * This subsystem controls the robot's intake mechanism, responsible for collecting game pieces.
 * It uses current-limited motor control to prevent brownouts and optimize performance during operation.
 * 
 * **Approach:**
 * - A single motor is configured with a smart current limit and voltage compensation to ensure consistent 
 *   performance under varying load conditions.
 * - Encoder feedback provides real-time monitoring of motor position, allowing for additional functionality 
 *   such as autonomous game piece collection.
 * - Flexible methods like `runIntake(speed)` and `stopIntake()` allow commands to control motor behavior dynamically.
 * 
 * **Why This Approach?**
 * - Current-limited control protects the motor and electrical system by preventing excessive power draw, 
 *   reducing the risk of system brownouts during high-demand operations.
 * - The use of telemetry for motor speed and current draw aids in debugging and performance tuning.
 * - Flexible methods and modular design make the subsystem easy to integrate into both teleop and autonomous routines.
 * 
 * **Improvements Made:**
 * - Introduced current-based motor control for more efficient and reliable operation.
 * - Added telemetry to log motor performance, assisting in debugging and tuning.
 * - Replaced hardcoded behaviors with parameterized methods for greater flexibility.
 * - Refactored autonomous intake logic to be non-blocking, ensuring compatibility with the command-based scheduler.
 */

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;
    private final DigitalInput irSensor; // IR sensor for game piece detection

    public IntakeSubsystem() {
        // Initialize motor
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();

        // Initialize IR sensor
        irSensor = new DigitalInput(0); // Example: DIO port 0

        // Configure motor settings
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(Constants.IntakeConstants.kIntakeIdleMode);
        intakeMotor.setSmartCurrentLimit(30);
        intakeMotor.enableVoltageCompensation(12.0);
        intakeMotor.burnFlash();

        resetEncoders();
    }

    @Override
    public void periodic() {
        // Update telemetry
        SmartDashboard.putNumber("Intake Encoder Position", getEncoderPosition());
        SmartDashboard.putNumber("Intake Motor Current", intakeMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Game Piece Detected", isGamePieceDetected());
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    public void resetEncoders() {
        intakeEncoder.setPosition(0);
    }

    public double getEncoderPosition() {
        return intakeEncoder.getPosition();
    }

    public boolean isGamePieceDetected() {
        // Check if the IR sensor beam is interrupted
        return !irSensor.get(); // Assuming 'false' indicates a beam interruption
    }
}
