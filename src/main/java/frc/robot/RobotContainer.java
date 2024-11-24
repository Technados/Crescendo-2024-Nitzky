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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.ArmControlCommand;
import frc.robot.commands.ArmHome;
import frc.robot.commands.AmpArm;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.commands.ShootSpeakerGroup;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShooter;
import frc.robot.commands.LobShooter;
import frc.robot.hijackablemovement.AprilTagLock;
import frc.robot.hijackablemovement.Joystick;
import frc.robot.hijackablemovement.MovementSource;
import frc.robot.commands.IntakeNote;

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

        // private final ShooterSubsystem m_lob = new ShooterSubsystem();

        private final ArmSubsystem m_arm = new ArmSubsystem();

        // private final LEDSubsystem m_led = new LEDSubsystem();

        // private final VisionSubsystem m_cam = new VisionSubsystem();

        // Commands
        private final ArmControlCommand m_armControlCommand = new ArmControlCommand(m_arm);

        private final IntakeNote intakeNoteCommand = new IntakeNote(m_intake);

        private final AmpArm ampArmCommand = new AmpArm(m_arm);

        private final ArmHome armHomeCommand = new ArmHome(m_arm);

        private final ScoreSpeaker scoreSpeakerCommand = new ScoreSpeaker(m_shooter);

        private final StartIntake startIntakeCommand = new StartIntake(m_intake);

        private final StopIntake stopIntakeCommand = new StopIntake(m_intake);

        private final StartShooter startShooterCommand = new StartShooter(m_shooter);

        private final StopShooter stopShooterCommand = new StopShooter(m_shooter);

        private final LobShooter lobShooterCommand = new LobShooter(m_shooter);

        private final ShootSpeakerGroup shootSpeakerParallelGroup = new ShootSpeakerGroup(m_intake, m_shooter);

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
        // Button A -
        // Button B -
        // Button X -
        // Button Y -
        // Start Button - Zero Robot Heading (resets forward)
        // **********************************************************

        // **********************************************************
        // Xbox Controller - 2/Operator - Port 1
        // ----------------------------------------------------------
        // Left Trigger - Move arm in (back to stow)
        // Right Trigger - Move arm out (open)
        // Left Bumper - Reverse intake
        // Right Bumper - Intake
        // Button A - Start shooter wheels
        // Button B -
        // Button X - Deploy arm to 'stow' rest position
        // Button Y - Deploy arm to amp scoring position
        // **********************************************************

        // The driver's controller
        public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        // The operator's controller
        public static XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

        private MovementSource hijackableRotation = new Joystick(); // get rotation from driver input;

        private MovementSource hijackableTranslation = new Joystick();

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
                NamedCommands.registerCommand("IntakeNote", intakeNoteCommand);
                NamedCommands.registerCommand("AmpArm", ampArmCommand);
                NamedCommands.registerCommand("ArmHome", armHomeCommand);
                NamedCommands.registerCommand("ScoreSpeaker", scoreSpeakerCommand);
                NamedCommands.registerCommand("ScoreSpeakerGroup", shootSpeakerParallelGroup);
                NamedCommands.registerCommand("StartIntake", startIntakeCommand);
                NamedCommands.registerCommand("StopIntake", stopIntakeCommand);
                NamedCommands.registerCommand("StartShooter", startShooterCommand);
                NamedCommands.registerCommand("StopShooter", stopShooterCommand);
                NamedCommands.registerCommand("LobShooter", lobShooterCommand);
                // set default arm command
                m_arm.setDefaultCommand(m_armControlCommand);

                // Configure default commands
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                m_robotDrive.setDefaultCommand(
                                new RunCommand(() -> m_robotDrive.drive(
                                                hijackableTranslation.getXSpeed(), // * 0.95
                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                OIConstants.kDriveDeadband), // * 0.95
                                                // -MathUtil.applyDeadband( m_driverController.getRightX(),
                                                // OIConstants.kDriveDeadband), // * 0.95
                                                hijackableRotation.getR(), // * 0.95
                                                isFieldRelative, true), m_robotDrive));

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
                                .onTrue(new InstantCommand(() -> hijackableRotation = new AprilTagLock()))
                                .onTrue(new InstantCommand(() -> hijackableTranslation = new AprilTagLock()))
                                .onTrue(new InstantCommand(() -> isFieldRelative = false))
                                .onFalse(new InstantCommand(() -> hijackableRotation = new Joystick()))
                                .onFalse(new InstantCommand(() -> hijackableTranslation = new Joystick()))
                                .onFalse(new InstantCommand(() -> isFieldRelative = true));

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
                                .onTrue(new InstantCommand(() -> m_intake.startIntake(), m_intake))
                                .onFalse(new InstantCommand(() -> m_intake.stopIntake(), m_intake));
                // Reverse Intake
                new JoystickButton(m_operatorController, Button.kLeftBumper.value)
                                .onTrue(new InstantCommand(() -> m_intake.reverseIntake(), m_intake))
                                .onFalse(new InstantCommand(() -> m_intake.stopIntake(), m_intake));

                // Shoot ring into speaker (pew pew!)
                new JoystickButton(m_operatorController, Button.kA.value)
                                .onTrue(new InstantCommand(() -> m_shooter.startShooter(), m_shooter))
                                .onFalse(new InstantCommand(() -> m_shooter.stopShooter(), m_shooter));

                // Reverse shooters (anti-pew pew!)
                new JoystickButton(m_operatorController, Button.kB.value)
                                .onTrue(new InstantCommand(() -> m_shooter.reverseShooter(), m_shooter))
                                .onFalse(new InstantCommand(() -> m_shooter.stopShooter(), m_shooter));

                new JoystickButton(m_operatorController, Button.kX.value)
                                .onTrue(new InstantCommand(() -> m_intake.resetEncoders(), m_intake));

                new JoystickButton(m_operatorController, Button.kY.value)
                                .onTrue(new InstantCommand(() -> m_shooter.startShooterLob(), m_shooter))
                                .onFalse(new InstantCommand(() -> m_shooter.stopShooter(), m_shooter));

                // set up arm preset positions
                // new JoystickButton(m_operatorController, XboxController.Button.kY.value)
                // .onTrue(new InstantCommand(() -> m_arm
                // .setTargetPosition(Constants.ArmConstants.kScoringPosition)));
                // new JoystickButton(m_operatorController, XboxController.Button.kX.value)
                // .onTrue(new InstantCommand(
                // () -> m_arm.setTargetPosition(Constants.ArmConstants.kHomePosition)));

                // Operator Arm control - Right trigger = deploy ; Left trigger = retract

                // SmartDashboard.putData("Test", new PathPlannerAuto("Test"));

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

                // If Alliance is blue reset odometry to the starting pose of the file
                if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                        m_robotDrive.resetOdometry(
                                        PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected()));
                }

                // Else retrieve the starting pose of the file and flip it to the red side
                else {
                        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected());
                        Translation2d translation = GeometryUtil.flipFieldPosition(startingPose.getTranslation());
                        Rotation2d rotation = GeometryUtil.flipFieldRotation(startingPose.getRotation());
                        m_robotDrive.resetOdometry(new Pose2d(translation.getX(), translation.getY(), rotation));
                }

                return AutoBuilder.buildAuto(autoChooser.getSelected());

        }

        // public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        // }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // The trajectory / auto paths below are the example trajectories included with
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////// the
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////// Rev
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////// MAXSwerve
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////// code
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////// template
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////// (unused)

        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        // AutoConstants.kMaxSpeedMetersPerSecond,
        // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(DriveConstants.kDriveKinematics);

        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(3, 0, new Rotation2d(0)),
        // config);
        //
        // var thetaController = new ProfiledPIDController(
        // AutoConstants.kPThetaController, 0, 0,
        // AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new
        // SwerveControllerCommand(
        // exampleTrajectory,
        // m_robotDrive::getPose, // Functional interface to feed supplier
        // DriveConstants.kDriveKinematics,

        // // Position controllers
        // new PIDController(AutoConstants.kPXController, 0, 0),
        // new PIDController(AutoConstants.kPYController, 0, 0),
        // thetaController,
        // m_robotDrive::setModuleStates,
        // m_robotDrive);

        // // // Reset odometry to the starting pose of the trajectory.
        // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
        // false, false));

        // Load the path you want to follow using its name in the GUI
        // PathPlannerPath path = PathPlannerPath.fromPathFile("Path");

        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.

        // Resets the robotOdometry to the currently selected path in the chooser
        // If blue, odometry will be reset to starting pose of the path
        // Else if red, odometry will be reset to the flipped starting pose of the blue
        // path

}
