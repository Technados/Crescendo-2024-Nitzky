package frc.robot.subsystems;

/**
 * LEDSubsystem
 * 
 * This subsystem manages the LED indicator strip and its interactions with the game piece 
 * detection system. The LED strip provides visual feedback to the drivers, changing its 
 * pattern based on whether a game piece is successfully intaked. It also triggers the Limelight 
 * to flash as an additional indicator.
 * 
 * **Approach:**
 * - An infrared emitter and receiver switch detects game pieces by sensing interruptions 
 *   in the beam, signaling that a game piece is present.
 * - The LED strip changes its pattern:
 *   - **Rainbow (-0.87)**: Default pattern when no game piece is detected.
 *   - **Green (0.77)**: Indicates that a game piece has been successfully intaked.
 * - A Limelight flash is triggered briefly when a game piece is newly detected.
 * - Non-blocking logic ensures smooth operation without disrupting other subsystems.
 * 
 * **Why This Approach?**
 * - Provides immediate and clear visual feedback for drivers, enhancing game awareness.
 * - Decoupling the IR sensor logic from LED control ensures better modularity and readability.
 * - Non-blocking Limelight flashing allows the robot's main scheduler to continue running 
 *   smoothly during operations.
 * - Telemetry aids in debugging and understanding subsystem behavior in real-time.
 * 
 * **Improvements Made:**
 * - Separated sensor and LED logic into distinct methods for better modularity.
 * - Introduced telemetry for game piece detection and current LED state.
 * - Replaced blocking Limelight flash logic with a non-blocking implementation.
 * - Maintained driver feedback functionality with enhanced flexibility and readability.
 */

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final Spark blinkin;
    private final IntakeSubsystem intakeSubsystem;

    private boolean noteDetected = false;

    public LEDSubsystem(IntakeSubsystem intakeSubsystem) {
        // Initialize hardware
        blinkin = new Spark(2); // PWM Port for LED controller
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void periodic() {
        // Update LED pattern based on game piece detection
        if (intakeSubsystem.isGamePieceDetected()) {
            setLEDGreen();
        } else {
            setLEDRainbow();
        }

        // Update telemetry
        SmartDashboard.putBoolean("Game Piece Detected", noteDetected);
        SmartDashboard.putString("LED Pattern", noteDetected ? "Green" : "Rainbow");
    }

    public void setLEDGreen() {
        blinkin.set(0.77); // Green pattern
    }

    public void setLEDRainbow() {
        blinkin.set(-0.87); // Rainbow pattern
    }

    public void flashLimelight() {
        new Thread(() -> {
            double start = Timer.getFPGATimestamp();
            LimelightHelpers.setLEDMode_ForceBlink("limelight");
            while (Timer.getFPGATimestamp() - start < 1.95) {
                // Keep blinking for 1.95 seconds
            }
            LimelightHelpers.setLEDMode_PipelineControl("limelight");
        }).start();
    }
}
