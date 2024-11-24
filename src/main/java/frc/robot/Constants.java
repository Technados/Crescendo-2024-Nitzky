// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/**
 * PDH Channel IDs
 * Left Front Drive Motor Controller: 12
 * Right Front Drive Motor Controller: 4
 * Right rear drive motor controller: 2
 * left rear drive motor controller: 15
 * Left Front steer Motor Controller: 13
 * Right Front steer Motor Controller: 3
 * Right rear steer motor controller: 1
 * left rear steer motor controller: 16
 * Intake Motor controller: 11
 * Right shooter motor controller: 10
 * Left shooter motor controller: 17
 * Limelight: 9
 * Right climber motor controller: 8
 * Left climber motor controller: 7
 */
package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
//import frc.lib.PIDGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class IntakeConstants {
    public static final int kIntakeMotorCanId = 11;
    public static final IdleMode kIntakeIdleMode = IdleMode.kBrake;

  }

  public static final class ShooterConstants {
    public static final int kLeftShooterMotorCanId = 9;
    public static final IdleMode kLeftShooterIdleMode = IdleMode.kBrake;

    public static final int kRightShooterMotorCanId = 10;
    public static final IdleMode kRightShooterIdleMode = IdleMode.kBrake;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.0;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 3.2; // 1.2 default - radians per second //peddie: 4.5
    public static final double kMagnitudeSlewRate = 3.5; // 1.8 default - percent per second (1 = 100%) //peddie: 4.5
    public static final double kRotationalSlewRate = 2.0; // 2.0 default - percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25.6);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.6);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 3;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 8;

    // gyro reversed boolean
    public static final boolean kGyroReversed = false;

  }

  public static final class ArmConstants {
    public static final int kRightArmCanId = 12;
    public static final int kLeftArmCanId = 13;

    public static final boolean kLeftArmMotorInverted = true;
    public static final boolean kRightArmMotorInverted = false;
    public static final int kArmMotorCurrentLimit = 40;

    public static final double kSoftLimitReverse = -1.15;
    public static final double kSoftLimitForward = 0.0;

    public static final double kArmGearRatio = (1.0 / 25) * (28 / 50) * (16 / 64);
    public static final double kArmPositionFactor = kArmGearRatio * 2 * Math.PI; // multiply sparkmax # by this to get
                                                                                 // arm position in radians
    public static final double kVelocityFactor = kArmGearRatio * (2 * Math.PI) / 60.0; // radians per second
    public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
    public static final double kArmZeroCosineOffset = 1.342; // radians to add to converted arm position to get
                                                             // real-world arm position (starts at
    // ~76.9deg angle)
    public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 3.0, 12.0 / kArmFreeSpeed, 0.0);
    // public static final PIDGains kArmPositionGains = new PIDGains(2.5, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(1.0, 2.0);

    public static final double kHomePosition = 0.0;
    public static final double kScoringPosition = 1.5;

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.07587;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04; // ORIGINALY AT 0.04
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05; // 0.05
    public static final int kOperatorControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5; // 3
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 0.3926991;// Math.PI / 4;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
