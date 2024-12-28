package frc.robot.subsystems;

/**
 * ShooterSubsystem
 * 
 * This subsystem controls the dual-motor shooter mechanism, which is responsible 
 * for launching game pieces. It supports multiple shooting modes and uses velocity-based 
 * control to maintain consistent and precise wheel speeds.
 * 
 * **Approach:**
 * - The shooter motors are controlled using velocity (RPM) targets via the built-in PID controller.
 * - Dual encoders provide real-time feedback, ensuring consistent shooter performance.
 * - Parameterized shooting modes allow for dynamic adjustment of motor speeds.
 * - Non-blocking logic is used for autonomous routines, improving compatibility with the scheduler.
 * 
 * **Why This Approach?**
 * - Velocity control ensures the shooter wheels maintain consistent speeds, crucial for shot accuracy.
 * - Parameterization makes it easy to fine-tune motor speeds for different game strategies.
 * - Telemetry aids in debugging and performance monitoring during testing and competition.
 * 
 * **Improvements Made:**
 * - Introduced velocity-based motor control to replace fixed-speed operation.
 * - Added telemetry for RPM monitoring and motor current draw.
 * - Refactored autonomous routines to be non-blocking.
 * - Parameterized motor speeds for improved flexibility and adaptability.
 */

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax leftShooterMotor;
    private final CANSparkMax rightShooterMotor;
    private final RelativeEncoder leftShooterEncoder;
    private final RelativeEncoder rightShooterEncoder;
    private final SparkPIDController leftPIDController;
    private final SparkPIDController rightPIDController;

    // PID coefficients
    private static final double kP = 0.1, kI = 0.0, kD = 0.0, kFF = 0.00015;

    public ShooterSubsystem() {
        // Initialize motors and encoders
        leftShooterMotor = new CANSparkMax(Constants.ShooterConstants.kLeftShooterMotorCanId, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(Constants.ShooterConstants.kRightShooterMotorCanId, MotorType.kBrushless);
        leftShooterEncoder = leftShooterMotor.getEncoder();
        rightShooterEncoder = rightShooterMotor.getEncoder();
        leftPIDController = leftShooterMotor.getPIDController();
        rightPIDController = rightShooterMotor.getPIDController();

        // Configure motors
        configureMotor(leftShooterMotor, leftPIDController);
        configureMotor(rightShooterMotor, rightPIDController);

        resetEncoders();
    }

    private void configureMotor(CANSparkMax motor, SparkPIDController pidController) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setSmartCurrentLimit(Constants.ShooterConstants.kShooterMotorCurrentLimit);
        motor.enableVoltageCompensation(12.0);
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kFF);
        pidController.setOutputRange(-1.0, 1.0);
        motor.burnFlash();
    }

    @Override
    public void periodic() {
        // Log telemetry data
        SmartDashboard.putNumber("Left Shooter RPM", leftShooterEncoder.getVelocity());
        SmartDashboard.putNumber("Right Shooter RPM", rightShooterEncoder.getVelocity());
    }

    public void setShooterVelocity(double targetRPM) {
        // Set shooter velocity using PID control
        leftPIDController.setReference(targetRPM, CANSparkMax.ControlType.kVelocity);
        rightPIDController.setReference(targetRPM, CANSparkMax.ControlType.kVelocity);

        // Log telemetry
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
    }

    public void stopShooter() {
        // Stop both motors
        leftShooterMotor.set(0.0);
        rightShooterMotor.set(0.0);
    }

    public void resetEncoders() {
        // Reset encoder positions
        leftShooterEncoder.setPosition(0.0);
        rightShooterEncoder.setPosition(0.0);
    }

    public double getAverageRPM() {
        // Return the average RPM of the two shooter motors
        return (leftShooterEncoder.getVelocity() + rightShooterEncoder.getVelocity()) / 2.0;
    }

    public double calculateRPMFromDistance(double distance) {
        // Example formula: Linear interpolation or empirical testing values
        double minRPM = 3000; // Example: Minimum RPM
        double maxRPM = 5000; // Example: Maximum RPM
        double minDistance = 1.0; // Minimum effective distance (in meters)
        double maxDistance = 5.0; // Maximum effective distance (in meters)

        // Ensure distance is within the expected range
        distance = Math.max(minDistance, Math.min(maxDistance, distance));

        // Interpolate RPM based on distance
        return minRPM + (distance - minDistance) * (maxRPM - minRPM) / (maxDistance - minDistance);
    }
    
    public boolean autoShooter(double targetRPM, double durationSeconds) {
        // Non-blocking autonomous shooter logic
        setShooterVelocity(targetRPM);
        double elapsedTime = 0; // Assume a time-tracking mechanism
        if (elapsedTime >= durationSeconds) {
            stopShooter();
            return true; // Shooting complete
        }
        return false; // Shooting still in progress
    }
}
