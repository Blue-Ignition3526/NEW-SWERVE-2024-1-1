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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveDriveIOReal;
import frc.robot.subsystems.SwerveDrive.SwerveDriveIOSim;
import frc.robot.subsystems.SwerveModule.SwerveModule;
import frc.robot.subsystems.SwerveModule.SwerveModuleIOSim;
import frc.robot.subsystems.SwerveModule.SwerveModuleIOSparkMaxPID;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIONavX;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.Intake.IntakeIOSim;


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

  public RobotContainer() {
    if (Robot.isReal()) {
      // Create all swerve modules and initialize
      this.m_frontLeft = new SwerveModule(new SwerveModuleIOSparkMaxPID(Constants.Swerve.Motors.kFrontLeftVars));
      this.m_frontRight = new SwerveModule(new SwerveModuleIOSparkMaxPID(Constants.Swerve.Motors.kFrontRightVars));
      this.m_backLeft = new SwerveModule(new SwerveModuleIOSparkMaxPID(Constants.Swerve.Motors.kBackLeftVars));
      this.m_backRight = new SwerveModule(new SwerveModuleIOSparkMaxPID(Constants.Swerve.Motors.kBackRightVars));

      // Create the swerve drive and initialize
      this.m_swerveDrive = new SwerveDrive(
        new SwerveDriveIOReal(m_frontLeft, m_frontRight, m_backLeft, m_backRight,
        new Gyro(new GyroIOPigeon(34)))
      );

      this.poseEstimator = new PoseEstimatorSubsystem(m_swerveDrive);

      // Create a new intake
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

      // Create a new intake
      this.m_intake = new Intake(new IntakeIOSim());

      Logger.recordMetadata("Robot", "Sim");
    }

    // Register all commands needed for Autonomous
    NamedCommands.registerCommand("IntakeIn", new IntakeIn(m_intake));
    NamedCommands.registerCommand("IntakeOut", new IntakeOut(m_intake));

    // Create a sendable chooser for the autonomous routines
    this.m_autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Command", this.m_autonomousChooser);
    
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
        true,
        () -> m_driverController.leftTrigger(0.1).getAsBoolean()
      )
    );
    
    m_driverController.x().whileTrue(new IntakeIn(m_intake));
    m_driverController.rightTrigger(0.1).whileTrue(new IntakeOut(m_intake));
  }

  /**
   * Returns the autonomous command to run. This will return null if no autonomous
   * @return Command
   */
  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}