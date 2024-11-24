package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {

    /** Creates a new Arm motor system. */

    private CANSparkMax leftArmMotor = new CANSparkMax(Constants.ArmConstants.kLeftArmCanId, MotorType.kBrushless);
    private CANSparkMax rightArmMotor = new CANSparkMax(Constants.ArmConstants.kRightArmCanId, MotorType.kBrushless);

    private static ArmSubsystem instance;

    // Encoders
    private RelativeEncoder leftArmEncoder = leftArmMotor.getEncoder();
    private RelativeEncoder rightArmEncoder = rightArmMotor.getEncoder();

    private float speed = 0.85f;
    private double value;

    public ArmSubsystem() {
        // create a new SPARK MAX for each arm motor and configure them
        leftArmMotor.restoreFactoryDefaults();
        rightArmMotor.restoreFactoryDefaults();

        // Resetting the encoder postion on robot startup

        rightArmMotor.setInverted(ArmConstants.kRightArmMotorInverted);
        leftArmMotor.setInverted(ArmConstants.kLeftArmMotorInverted);
        rightArmMotor.setInverted(ArmConstants.kRightArmMotorInverted);
        leftArmMotor.setInverted(ArmConstants.kLeftArmMotorInverted);
        rightArmMotor.setInverted(ArmConstants.kRightArmMotorInverted);
        leftArmMotor.setInverted(ArmConstants.kLeftArmMotorInverted);
        rightArmMotor.setSmartCurrentLimit(Constants.ArmConstants.kArmMotorCurrentLimit);
        leftArmMotor.setSmartCurrentLimit(Constants.ArmConstants.kArmMotorCurrentLimit);
        rightArmMotor.setIdleMode(IdleMode.kBrake);
        leftArmMotor.setIdleMode(IdleMode.kBrake);

        leftArmEncoder.setPosition(0);
        rightArmEncoder.setPosition(0);

        rightArmMotor.burnFlash();
        leftArmMotor.burnFlash();

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // int num = (int) (value * 100);
        // String percent = String.valueOf(num);
        // SmartDashboard.putString("Arm Motors Speed", percent + "%");
        // SmartDashboard.putNumber("Left Arm Encoder Position",
        // getLeftEncoderPosition());
        // SmartDashboard.putNumber("Right Arm Encoder Position",
        // getRightEncoderPosition());
        // SmartDashboard.putNumber("Average Arm Encoder Postion",
        // getAverageArmEncoderDistance());
    }

    // Returns an instance of this subsystem
    public static ArmSubsystem getInstance() {
        if (instance == null) {
            instance = new ArmSubsystem();
        }
        return instance;
    }

    // Resets the encoders position to 0.0
    public void resetEncoders() {
        leftArmEncoder.setPosition(0.0);
        rightArmEncoder.setPosition(0.0);
    }

    // Returns the left arm encoder position
    public double getLeftEncoderPosition() {
        return leftArmEncoder.getPosition();
    }

    // Returns the right arm encoder position
    public double getRightEncoderPosition() {
        return rightArmEncoder.getPosition();
    }

    // Returns the average encoder distance of left and right encoders
    public double getAverageArmEncoderDistance() {
        return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
    }

    // Returns true when the encoder right arm is at it's limit
    public boolean getRightTriggerLimit() {
        return ((leftArmEncoder.getPosition() > 63.5) ||
                (rightArmEncoder.getPosition() > 63.5));
    }

    // Returns true when the encoder left arm is at it's limit
    public boolean getLeftTriggerLimit() {
        return ((leftArmEncoder.getPosition() < 2) ||
                (rightArmEncoder.getPosition() < 2));
    }

    // Controls arm movement based on trigger inputs
    public void moveArm(double leftTrigger, double rightTrigger) {
        this.value = value;

        if (getLeftTriggerLimit()) {
            leftTrigger = 0;
        } else if (getRightTriggerLimit()) {
            rightTrigger = 0;
        }

        if (leftTrigger != 0) {
            value = leftTrigger;
        } else if (rightTrigger != 0) {
            value = rightTrigger;
        } else {
            value = 0;
        }

        rightArmMotor.set(value);
        leftArmMotor.set(value);
    }

    // Spins the arm motors forwards
    public void armForward() {
        leftArmMotor.set(-speed);
        rightArmMotor.set(-speed);
    }

    // Spins the arm motors in reverse
    public void armReverse() {
        leftArmMotor.set(speed);
        rightArmMotor.set(speed);
    }

    // Stops the arm motors
    public void stopArm() {
        leftArmMotor.set(0.0);
        rightArmMotor.set(0.0);
    }

    // Auto method to extend the arm out
    public boolean autoArmOut() {
        resetEncoders();
        while ((leftArmEncoder.getPosition() <= 65) || (rightArmEncoder.getPosition() <= 65)) {
            armForward();
        }
        stopArm();
        return true;
    }

    // Auto method to extend the arm in
    public boolean autoArmIn() {
        resetEncoders();
        while ((leftArmEncoder.getPosition() >= 0) || (rightArmEncoder.getPosition() >= 0)) {
            armReverse();
        }
        stopArm();
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
