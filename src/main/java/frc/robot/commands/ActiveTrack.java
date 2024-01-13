// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Physical;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionSubsystem;


public class ActiveTrack extends Command {
  /**
   * Swerve drive subsystem
   */
  SwerveDrive m_swerveDrive;

  /**
   * X speed supplier
   */
  Supplier<Double> xSpeed;

  /**
   * Y speed supplier
   */
  Supplier<Double> ySpeed;

  /**
   * Rotation speed supplier
   */
  Supplier<Double> rotSpeed;

  /**
   * Whether the drive is field relative
   */
  Supplier<Boolean> fieldRelative;

  /**
   * Slew rate limiters
   */
  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.Swerve.Physical.kTeleopMaxAccelerationUnitsPerSecond);

  /**
   * Slew rate limiters
   */
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.Swerve.Physical.kTeleopMaxAccelerationUnitsPerSecond);

  /**
   * Slew rate limiters
   */
  SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.Swerve.Physical.kTeleopMaxAngularAccelerationUnitsPerSecond);

  /**
   * Vision subsystem
   */
  VisionSubsystem vision;

  /**
   * Creates a new DriveSwerve command.
   * @param m_swerveDrive The swerve drive subsystem
   * @param vision The vision subsystem
   * @param x The x speed supplier
   * @param y The y speed supplier
   * @param rot The rotaion speed if there is no AprilTag found
   * @param fieldRelative Whether the drive is field relative
   */
  public ActiveTrack(SwerveDrive m_swerveDrive, VisionSubsystem vision, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot, Supplier<Boolean> fieldRelative) {
    this.m_swerveDrive = m_swerveDrive;
    this.xSpeed = x;
    this.ySpeed = y;
    this.rotSpeed = rot;
    this.fieldRelative = fieldRelative;
    this.vision = vision;
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the current speeds
    double xSpeed = this.xSpeed.get();
    double ySpeed = this.ySpeed.get();
    double rotSpeed;

    boolean tagIsVisible;
    
    Optional<Double> rotSpeedOpt = vision.calculateRotSpeed(Constants.Vision.AprilTags.kSpeakerTagID);
    if (rotSpeedOpt.isPresent()){
      rotSpeed = rotSpeedOpt.get();
      tagIsVisible = true;
    }else{
      rotSpeed = this.rotSpeed.get();
      tagIsVisible = false;
    }
    if(tagIsVisible) Logger.recordOutput("CalculatedRotationalSpeed", rotSpeed);

    
    // If the speeds are lower than the deadzone
    xSpeed = Math.abs(xSpeed) > Constants.Operator.kDeadzone ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.Operator.kDeadzone ? ySpeed : 0.0;
    if (!tagIsVisible) rotSpeed = Math.abs(rotSpeed) > Constants.Operator.kDeadzone ? rotSpeed : 0.0;

    // Multiply by the top speed
    xSpeed = xLimiter.calculate(xSpeed) * Physical.kTeleopMaxSpeedMetersPerSecond / 2;
    ySpeed = yLimiter.calculate(ySpeed) * Physical.kTeleopMaxSpeedMetersPerSecond / 2;
    rotSpeed = rotLimiter.calculate(rotSpeed) * Physical.kTeleopMaxAngularSpeedRadiansPerSecond / 2;

    // Drive the robot
    if (this.fieldRelative.get()) {
      m_swerveDrive.driveFieldRelative(xSpeed, ySpeed, rotSpeed);
    } else {
      m_swerveDrive.driveRobotRelative(xSpeed, ySpeed, rotSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
