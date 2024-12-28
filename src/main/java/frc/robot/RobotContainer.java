// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Code written by R. Nitzky, L. DuPont, A. Peralta, J. Tineo

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.CombinedAimAndRangeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.ManualArmControlCommand;
import frc.robot.commands.ShooterCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        // The robot's subsystems

	private final DriveSubsystem m_robotDrive = new DriveSubsystem();
	private final IntakeSubsystem m_intake = new IntakeSubsystem();
	private final ShooterSubsystem m_shooter = new ShooterSubsystem();
	private final ArmSubsystem m_arm = new ArmSubsystem();
	private final LimelightSubsystem m_limelight = new LimelightSubsystem();
	private final LEDSubsystem m_led = new LEDSubsystem(m_intake);


        // Commands
	
	private final IntakeCommand forwardIntakeCommand = new IntakeCommand(m_intake, 20.0); // Forward at 20 amps
	private final IntakeCommand reverseIntakeCommand = new IntakeCommand(m_intake, -20.0); // Reverse at 20 amps;
	private final ShooterCommand shooterCommand = new ShooterCommand(m_shooter, 4000); // Target RPM
	private final ArmCommand armCommand = new ArmCommand(m_arm, 50.0); // Example position
	private final LimelightCommand limelightCommand = new LimelightCommand(m_limelight, m_robotDrive);


        // create autoChooser
        private final SendableChooser<String> autoChooser = new SendableChooser<>();

        private boolean isFieldRelative = true;

        // Controller Buttons Used

        // **********************************************************
        // Xbox Controller - 1/Driver - Port 0
        // ----------------------------------------------------------
        // Left Stick (Translate along X and Y plane) - "Swerve"
        // Right Stick (Rotate about Z-Axis) - Drive Right/Left
        // Left Trigger -
        // Right Trigger -
        // Left Bumper -
        // Right Bumper - Hold to brake (locks wheels in 'X' pattern to prevent
        // movement)
        // Button A - while held, engages the auto aim and range feature.  Return to normal drive when released
        // Button B -
        // Button X -
        // Button Y -
        // Start Button - Zero Robot Heading (manually resets forward for field relative if heading is off)
        // **********************************************************

        // **********************************************************
        // Xbox Controller - 2/Operator - Port 1
        // ----------------------------------------------------------
        // Left Trigger - While held, Move arm in (back to stow)
        // Right Trigger - while held, Move arm out (open)
        // Left Bumper - while held, Reverse intake
        // Right Bumper - while held, Forward Intake
        // Button A - while held, Start shooter wheels
        // Button B -
        // Button X - Deploy arm to 'stow' min position
        // Button Y - Deploy arm to amp max position
        // **********************************************************

        // The driver's controller
        public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        // The operator's controller
        public static XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Command Groups
        // SequentialCommandGroup intake = new SequentialCommandGroup(
        // new IntakeNote(m_intake)
        // )

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Register named commands for PathPlanner GUI
                NamedCommands.registerCommand("Shooter", shooterCommand);
                NamedCommands.registerCommand("Intake", forwardIntakeCommand);
      
                // set default arm command
                m_arm.setDefaultCommand(armCommand);

                // Configure default commands
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                m_robotDrive.setDefaultCommand(
    new RunCommand(() -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), // Translation Y
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), // Translation X
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband), // Rotation
        isFieldRelative, true), // Field-relative control
        m_robotDrive)
);



                // set the arm subsystem to run the "runAutomatic" function continuously when no
                // other command
                // is running
                // m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                // Adding autos to the autoChooser
                // First arg if the name that will show in shuffleboard
                // Second arg is the name of the auto inside of PathPlannerGUI

                autoChooser.addOption("CenterTwoPiece", "CenterTwoPiece");
                autoChooser.addOption("AmpSideShootMid", "AmpSideShootMid");
                autoChooser.addOption("Turning", "Turning");
                autoChooser.addOption("CenterFourPiece", "CenterFourPiece");
                autoChooser.addOption("NewCenterFourPiece", "NewCenterFourPiece");
                autoChooser.addOption("CenterThreePieceAmpSide", "CenterThreePieceAmpSide");
                autoChooser.addOption("CenterThreePieceSourceSide", "CenterThreePieceSourceSide");
                autoChooser.addOption("AmpSideTwoPiece", "AmpSideTwoPiece");
                autoChooser.addOption("OutTheWay", "OutTheWay");
                autoChooser.addOption("OutTheWayMid", "OutTheWayMid");
                autoChooser.addOption("CenterShootStay", "CenterShootStay");
                autoChooser.addOption("AmpSideShootStay", "AmpSideShootStay");
                autoChooser.addOption("SourceSideShootStay", "SourceSideShootStay");
                autoChooser.addOption("KPTuning", "KPTuning");
                autoChooser.addOption("Squiggle", "Squiggle");
                autoChooser.addOption("none", null);

                // Creating a new shuffleboard tab and adding the autoChooser
                Shuffleboard.getTab("PathPlanner Autonomous").add(autoChooser);
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                //////////////////////////////////////////////////////////////////////////////////////////////////
                ///////////////////////////////// Driver Controller commands
                ////////////////////////////////////////////////////////////////////////////////////////////////// /////////////////////////////////////
                // Lock wheels in 'X' pattern
                new JoystickButton(m_driverController, Button.kRightBumper.value)
                                .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

                // Pressing 'Start' button will zero the heading of the robot (reset what is
                // 'forward')
                new JoystickButton(m_driverController, Button.kStart.value)
                                .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

                // Pressing the 'A' button will initiate rotation of the robot to align with
                // April tag using LL data
                new JoystickButton(m_driverController, Button.kA.value)
                        .onTrue(new CombinedAimAndRangeCommand(m_limelight, m_robotDrive, m_shooter)) // Engage auto aim and range
                        .onFalse(new InstantCommand(() -> isFieldRelative = true)); // Reset field-relative control on release
            

                // Hold the left bumper to enable low speed mode when crossing the field
                // (currently: 2.5)
                // High speed mode enabled by default (currently (4.0))
                new JoystickButton(m_driverController, Button.kLeftBumper.value)
                                .onTrue(new InstantCommand(() -> m_robotDrive.setLowSpeed(), m_robotDrive))
                                .onFalse(new InstantCommand(() -> m_robotDrive.setHighSpeed(), m_robotDrive));
                //////////////////////////////////////////////////////////////////////////////////////////////////
                ///////////////////////////////// Operator Controller commands
                ////////////////////////////////////////////////////////////////////////////////////////////////// /////////////////////////////////////

                // Ring Intake In on floor
                new JoystickButton(m_operatorController, Button.kRightBumper.value)
                                .onTrue(new InstantCommand(() -> m_intake.runIntakeAtCurrent(1), m_intake))
                                .onFalse(new InstantCommand(() -> m_intake.stopIntake(), m_intake));
                // Reverse Intake
                new JoystickButton(m_operatorController, Button.kLeftBumper.value)
                                .onTrue(new InstantCommand(() -> m_intake.runIntakeAtCurrent(1), m_intake))
                                .onFalse(new InstantCommand(() -> m_intake.stopIntake(), m_intake));

                // Shoot ring into speaker (pew pew!)
                new JoystickButton(m_operatorController, Button.kA.value)
                                .onTrue(new InstantCommand(() -> m_shooter.setShooterVelocity(1), m_shooter))
                                .onFalse(new InstantCommand(() -> m_shooter.stopShooter(), m_shooter));

                // Reverse shooters (anti-pew pew!)
                new JoystickButton(m_operatorController, Button.kB.value)
                                .onTrue(new InstantCommand(() -> m_shooter.setShooterVelocity(1), m_shooter))
                                .onFalse(new InstantCommand(() -> m_shooter.stopShooter(), m_shooter));

                new JoystickButton(m_operatorController, Button.kX.value)
                                .onTrue(new InstantCommand(() -> m_intake.resetEncoders(), m_intake));

        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        // The command below resets robot odometry and checks which alliance (red/blue)
        // is selected in the driver station--
        // If the alliance chosen is NOT BLUE - the pathplanner path will flip to red
        // using the .flipPath() method

	public Command getAutonomousCommand() {
   	 if (autoChooser.getSelected() == null) {
        return null;
    	}

    	Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected());
    	if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        m_robotDrive.resetOdometry(startingPose);
    	} else {
        Translation2d flippedTranslation = GeometryUtil.flipFieldPosition(startingPose.getTranslation());
        Rotation2d flippedRotation = GeometryUtil.flipFieldRotation(startingPose.getRotation());
        m_robotDrive.resetOdometry(new Pose2d(flippedTranslation, flippedRotation));
   	 }

    	return AutoBuilder.buildAuto(autoChooser.getSelected());
}



}
