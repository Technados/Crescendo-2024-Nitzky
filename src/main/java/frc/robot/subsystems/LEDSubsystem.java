package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.utils.LimelightHelpers;

public class LEDSubsystem {

    // PWM Port 2
    private Spark Blinkin = new Spark(2);

    // DIO Port 0
    private DigitalInput hasNote = new DigitalInput(0);

    private boolean state = true;

    public void controlLED() {

        // If sensor is active
        //
        if (hasNote.get() == false) {
            Blinkin.set(0.77);
        } else {
            // -0.87
            Blinkin.set(-0.87);
            state = true;

        }
        if (hasNote.get() == false && state == true) {
            state = false;
            flashLimelight();

        }
    }

    public void flashLimelight() {
        double start = Timer.getFPGATimestamp();
        LimelightHelpers.setLEDMode_ForceBlink("limelight");
        while (true) {
            double currentTime = Timer.getFPGATimestamp();
            if (currentTime - start > 1.95) {
                LimelightHelpers.setLEDMode_PipelineControl("limelight");
                break;
            }
        }

    }
}
