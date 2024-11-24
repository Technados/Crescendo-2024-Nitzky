// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// find unused imports at the bottom of this file (may need later)
//import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
    // Odometry calculates the robot's position on the field based on module states and gyro data.
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

public class DriveSubsystem extends SubsystemBase {
    // This class handles the robot's swerve drive functionality, including odometry and movement.

  private final MAXSwerveModule[] swerveModules;
  private SwerveModulePosition[] swerveModulePositions;

  private double maxSpeedMPS = Constants.DriveConstants.kMaxSpeedMetersPerSecond;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    // Define each swerve module with its corresponding CAN IDs and angular offsets.
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    // Define each swerve module with its corresponding CAN IDs and angular offsets.
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
    // Define each swerve module with its corresponding CAN IDs and angular offsets.
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
    // Define each swerve module with its corresponding CAN IDs and angular offsets.
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  ///////////////////// The gyro sensor
  ///////////////////// //////////////////////////////////////////////
  // Next line is gyro setup for NavX-2 Micro gyro from Kauai Labs
  //// Additional change: since using NavX-2 gyro, all getAngle calls in the drive
  // sub system had to be chnaged to negative values
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);
    // The NavX gyro is used to track the robot's orientation on the field.

  ////////////////////////////////////////////////////////////////////////////////////

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  //////////////////////////////////////////////////// Odometry
  //////////////////////////////////////////////////// /////////////////////////////////////////////
  // Odometry class for tracking robot pose

  // public void addVisionMeasurement(Pose2d botpose, double d, Vector<N3> fill) {
  // }

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    // Odometry calculates the robot's position on the field based on module states and gyro data.
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
    // The NavX gyro is used to track the robot's orientation on the field.
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
  // below may be used as drop in replacement for above obometry class - to alow
  // addition of
  // vision/limelight data to update odometry (update imports to use)

  // SwerveDrivePoseEstimator m_PoseEstimator = new SwerveDrivePoseEstimator(
  // DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(-m_gyro.getAngle()),
    // The NavX gyro is used to track the robot's orientation on the field.
  // new SwerveModulePosition[] {
  // m_frontLeft.getPosition(),
  // m_frontRight.getPosition(),
  // m_rearLeft.getPosition(),
  // m_rearRight.getPosition()
  // },
  // new Pose2d());
  //////////////////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** Creates a new DriveSubsystem. */
    // This class handles the robot's swerve drive functionality, including odometry and movement.
  public DriveSubsystem() {
    // This class handles the robot's swerve drive functionality, including odometry and movement.
    // Do all subsystem initialization here
    // ...
    swerveModules = new MAXSwerveModule[] { m_frontLeft, m_frontRight, m_rearLeft, m_rearRight };
    // Define each swerve module with its corresponding CAN IDs and angular offsets.
    swerveModulePositions = new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(),
    // Retrieve the current positions of the swerve modules for odometry updates.
        m_rearLeft.getPosition(), m_rearRight.getPosition() };

    // it is recommended to configure AutoBuilder at the end of your drive
    // subsystem's constructor
    // Configure the AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
    // Retrieves the robot's current estimated position on the field.
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    // Resets the robot's position on the field to a specified pose.
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(3.50, 0.0, 0.11), // Translation PID constants
            new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            0.32385, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig()), // Default path replanning config. See the API for the options here

        // Boolean supplier that controls when the path will be mirrored for the red
        // alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, this); // Reference to this subsystem to set requirements
  }

  @Override
  public void periodic() {
    // Periodic updates include odometry and data sent to the SmartDashboard.
    SmartDashboard.putNumber("Gyro", getHeading()); // returns the heading of the robot and sends to dashboard

    // for (int i = 0; i < 4; i++) {
    // swerveModulePositions[i] = swerveModules[i].getPosition();
    // swerveModules[i].putSmartDashboard();
    // }

    // Update the odometry in the periodic block
    // Periodic updates include odometry and data sent to the SmartDashboard.
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
    // The NavX gyro is used to track the robot's orientation on the field.
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()

        });

    SmartDashboard.putNumber("2DPose X", m_odometry.getPoseMeters().getX()); //
    // Retrieves the robot's current estimated position on the field.
    // gets the field relative 'x' position of
    // the robot and send to dashboard
    SmartDashboard.putNumber("2DPose Y", m_odometry.getPoseMeters().getY()); //
    // Retrieves the robot's current estimated position on the field.
    // gets the field relative 'y' position of
    // the robot and send to dashboard

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // Retrieves the robot's current estimated position on the field.
    return m_odometry.getPoseMeters();
    // Retrieves the robot's current estimated position on the field.
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    // Resets the robot's position on the field to a specified pose.
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
    // The NavX gyro is used to track the robot's orientation on the field.
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);

  }

  //////////////////////////////////////////////// Drive Method
  //////////////////////////////////////////////// /////////////////////////////////////////
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    // Method to control robot movement using speed inputs and optional field-relative control.

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * maxSpeedMPS;
    double ySpeedDelivered = ySpeedCommanded * maxSpeedMPS;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getAngle()))
    // The NavX gyro is used to track the robot's orientation on the field.
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeedMPS);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

  }

  /**
   * Sets the wheels into an X formation to prevent movement. (hold right bumper
   * on driver controller)
   */
  public void setX() {
    // Locks the robot's wheels in an X formation to prevent unwanted movement.
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, maxSpeedMPS);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  // start button on driver controller
  public void zeroHeading() {
    m_gyro.reset();
    // The NavX gyro is used to track the robot's orientation on the field.
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
    // The NavX gyro is used to track the robot's orientation on the field.
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();
    return states;
  }

  // returns the robot relative chassis speess
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  //
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Method to control robot movement using speed inputs and optional field-relative control.
    // set blow rate limitting to false for PathPlanner because it applies
    // acceleration constants
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, false);

  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    // The NavX gyro is used to track the robot's orientation on the field.
  }

  public void setHighSpeed() {
    maxSpeedMPS = 4.0;
  }

  public void setLowSpeed() {
    maxSpeedMPS = 1.5;
  }

}
