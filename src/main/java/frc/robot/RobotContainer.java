// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ActiveTrackLimeLight;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.IntakeOut;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveDriveIOReal;
import frc.robot.subsystems.SwerveDrive.SwerveDriveIOSim;
import frc.robot.subsystems.SwerveModule.SwerveModule;
import frc.robot.subsystems.SwerveModule.SwerveModuleIOReal;
import frc.robot.subsystems.SwerveModule.SwerveModuleIOSim;
import frc.robot.subsystems.LimeLightPose;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOReal;


/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
  /**
   * Main driver controller
   */
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  /**
   * Front Left Swerve Module
   */
  SwerveModule m_frontLeft;

  /**
   * Front Right Swerve Module
   */
  SwerveModule m_frontRight;

  /**
   * Back Left Swerve Module
   */
  SwerveModule m_backLeft;

  /**
   * Back Right Swerve Module
   */
  SwerveModule m_backRight;

  /**
   * Swerve Drivetrain
   */
  SwerveDrive m_swerveDrive;

  /**
   * Intake
   */
  Intake m_intake;

  /**
   * Autonomous Command Chooser
   */
  SendableChooser<Command> m_autonomousChooser;

  PoseEstimatorSubsystem poseEstimator;

  LimeLightPose lime_poseEstimator;

  public RobotContainer() {
    if (Robot.isReal()) {
      // Create all swerve modules and initialize
      this.m_frontLeft = new SwerveModule(new SwerveModuleIOReal(Constants.Swerve.Motors.kFrontLeftVars));
      this.m_frontRight = new SwerveModule(new SwerveModuleIOReal(Constants.Swerve.Motors.kFrontRightVars));
      this.m_backLeft = new SwerveModule(new SwerveModuleIOReal(Constants.Swerve.Motors.kBackLeftVars));
      this.m_backRight = new SwerveModule(new SwerveModuleIOReal(Constants.Swerve.Motors.kBackRightVars));

      // Create the swerve drive and initialize
      this.m_swerveDrive = new SwerveDrive(new SwerveDriveIOReal(m_frontLeft, m_frontRight, m_backLeft, m_backRight));

      this.m_intake = new Intake(new IntakeIOReal(30, 31));

      Logger.recordMetadata("Robot", "Real");
    } else {
      // Create all swerve modules and initialize
      this.m_frontLeft = new SwerveModule(new SwerveModuleIOSim(Constants.Swerve.Motors.kFrontLeftVars));
      this.m_frontRight = new SwerveModule(new SwerveModuleIOSim(Constants.Swerve.Motors.kFrontRightVars));
      this.m_backLeft = new SwerveModule(new SwerveModuleIOSim(Constants.Swerve.Motors.kBackLeftVars));
      this.m_backRight = new SwerveModule(new SwerveModuleIOSim(Constants.Swerve.Motors.kBackRightVars));

      // Create the swerve drive and initialize
      this.m_swerveDrive = new SwerveDrive(new SwerveDriveIOSim(m_frontLeft, m_frontRight, m_backLeft, m_backRight));

      Logger.recordMetadata("Robot", "Sim");
    }

    // Register all commands needed for Autonomous
    NamedCommands.registerCommand("IntakeIn", new WaitCommand(1));
    NamedCommands.registerCommand("IntakeOut", new WaitCommand(1));

    // Create a sendable chooser for the autonomous routines
    SendableChooser<Command> m_autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Command", m_autonomousChooser);
    
    // Configure the controller bindings
    configureBindings();
  }

  
  private void configureBindings() {
    // By default drive the swerve drive forever and update with the joystick values
    m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getRightX(),
        () -> !m_driverController.rightBumper().getAsBoolean()
      )
    );

    // Active track when the left trigger is pressed
    m_driverController.leftTrigger(0.1).whileTrue(new ActiveTrackLimeLight(
      m_swerveDrive,
      lime_poseEstimator,
      () -> m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX(),
      () -> -m_driverController.getRightX(),
      () -> !m_driverController.rightBumper().getAsBoolean()
    ));

    // When the right trigger is pressed, drive the swerve drive forward
    m_driverController.rightTrigger(0.1).whileTrue(new DriveSwerve(
      m_swerveDrive,
      () -> m_driverController.getRightTriggerAxis(),
      () -> 0.0,
      () -> 0.0,
      () -> !m_driverController.rightBumper().getAsBoolean()
    ));

    m_driverController.y().whileTrue(new IntakeOut(m_intake));
  }

  /**
   * Returns the autonomous command to run. This will return null if no autonomous
   * @return Command
   */
  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}