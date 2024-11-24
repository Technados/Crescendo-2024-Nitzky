
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.cameraserver.CameraServer;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem instance;
  /** Creates a new Shooter. */

  // Motor Controllers
  private CANSparkMax leftShooterMotor = new CANSparkMax(Constants.ShooterConstants.kLeftShooterMotorCanId,
      MotorType.kBrushless);
  private CANSparkMax rightShooterMotor = new CANSparkMax(Constants.ShooterConstants.kRightShooterMotorCanId,
      MotorType.kBrushless);

  // Relative Encoders
  private RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();
  private RelativeEncoder rightShooterEncoder = rightShooterMotor.getEncoder();

  // Stores the speed of the Shooter motor
  private float leftShooterSpeed = 0.85f;
  private float rightShooterSpeed = 0.85f;

  // Slowe shooter speed for lob shots
  private float leftShooterSpeedLob = 0.60f;
  private float rightShooterSpeedLob = 0.60f;

  // private float lowSpeed = 0.65f;

  public ShooterSubsystem() {

    // Resets the motors by restoring factory default settings
    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();

    // Sets the right side motors to be inverted
    // Inversion below is repeated because sometimes the 'setinverted' does not
    // "stick" in the motor controller...
    leftShooterMotor.setInverted(true);
    rightShooterMotor.setInverted(false);
    leftShooterMotor.setInverted(true);
    rightShooterMotor.setInverted(false);
    leftShooterMotor.setInverted(true);
    rightShooterMotor.setInverted(false);

    // Sets idle mode of the motor controllers to brake mode
    leftShooterMotor.setIdleMode(ShooterConstants.kLeftShooterIdleMode);
    rightShooterMotor.setIdleMode(ShooterConstants.kRightShooterIdleMode);

    // Resetting the encoder postion on robot startup
    leftShooterEncoder.setPosition(0);
    rightShooterEncoder.setPosition(0);

    leftShooterMotor.burnFlash();
    rightShooterMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // int num = (int) (leftShooterSpeed * 100);
    // String percent = String.valueOf(num);
    // SmartDashboard.putString("Shooter Motors Speed", percent + "%");
  }

  // Returns an instance of this subsystem
  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  // Spins the shooter motors forwards
  public void startShooter() {
    leftShooterMotor.set(leftShooterSpeed);
    rightShooterMotor.set(rightShooterSpeed);
  }

  // Spins the shooter motors for lob shots
  public void startShooterLob() {
    leftShooterMotor.set(leftShooterSpeedLob);
    rightShooterMotor.set(rightShooterSpeedLob);
  }

  // Spins the shooter motors in reverse
  public void reverseShooter() {
    leftShooterMotor.set(-leftShooterSpeed * 0.5);
    rightShooterMotor.set(-rightShooterSpeed * 0.5);
  }

  // Stops the shooter motors
  public void stopShooter() {
    leftShooterMotor.set(0.0);
    rightShooterMotor.set(0.0);
  }

  // Resets the position of the encoders to 0.0
  public void resetEncoders() {
    rightShooterEncoder.setPosition(0.0);
    leftShooterEncoder.setPosition(0.0);
  }

  // Returns the position of the left shooter encoder
  public double getLeftEncoderPosition() {
    return leftShooterEncoder.getPosition();
  }

  // Returns the position of the right shooter encoder
  public double getRightEncoderPosition() {
    return rightShooterEncoder.getPosition();
  }

  // Returns the average encoder distance of left and right encoders
  public double getAverageEncoderDistance() {
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }

  // Auto method that spins the shooter until it reaches a set encoder position,
  // causing the motor to stop.
  public boolean autoShooter() {
    resetEncoders();
    while (getAverageEncoderDistance() <= 200.0) {
      // value of 200 is arbitrary -- will test how long shooter runs for with this
      // value and change as needed
      startShooter();
    }
    stopShooter();
    return true;
  }
}
