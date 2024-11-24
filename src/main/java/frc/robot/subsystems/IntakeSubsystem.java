
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import frc.robot.subsystems.LEDSubsystem;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
  private final LEDSubsystem m_led = new LEDSubsystem();
  /** Creates a new Intake. */

  private static IntakeSubsystem instance;

  // Motor Controllers
  private CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);

  // Relative Encoders
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  // Stores the speed of the intake motor
  private float intakeRingSpeed = 0.90f;
  // private float lowSpeed = 0.65f;

  public IntakeSubsystem() {
    // Reset the motors
    intakeMotor.restoreFactoryDefaults();

    // Sets the intake motor to be inverted
    intakeMotor.setInverted(false);
    intakeMotor.setInverted(false);

    // Sets idle mode of the motor controllers to brake mode
    intakeMotor.setIdleMode(IntakeConstants.kIntakeIdleMode);

    intakeMotor.burnFlash();

    // Resetting the encoder postion on robot startup
    resetEncoders();
  }

  @Override
  public void periodic() {
    m_led.controlLED();

  }

  // Returns an instance of this subsystem
  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  // Spins the intake motors forwards
  public void startIntake() {
    intakeMotor.set(intakeRingSpeed);
  }

  // Spins the intake motors reverse
  public void reverseIntake() {
    intakeMotor.set(-intakeRingSpeed);
  }

  // Stops the intake motors
  public void stopIntake() {
    intakeMotor.set(0.0);
  }

  // Resets the position of the intake encoder to 0.0
  public void resetEncoders() {
    intakeEncoder.setPosition(0);
  }

  // Returns the encoder position
  public double getEncoderPosition() {
    return intakeEncoder.getPosition();
  }

  // Auto method for intaking a game piece
  public boolean autoIntake() {
    resetEncoders();
    while (getEncoderPosition() <= 50.0) {
      // value of 50.0 is arbitrary -- will test how long shooter runs for with this
      // value and change as needed
      startIntake();
    }
    stopIntake();
    return true;
  }
}
