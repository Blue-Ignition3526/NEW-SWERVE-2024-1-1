// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  SwerveDriveIO io;
  SwerveDriveIOInputsAutoLogged inputs = new SwerveDriveIOInputsAutoLogged();

  /** Creates a new SwerveDrive. */
  public SwerveDrive(SwerveDriveIO io) {
    this.io = io;
    io.configureAutoBuilder(this);
  }

  public Rotation2d getRotation2d() {
    return io.getRotation2d();
  }

  public SwerveModuleState[] getModuleStates() {
    return io.getModuleStates();
  }

  public SwerveDriveOdometry getOdometry() {
    return io.getOdometry();
  }

  public Pose2d getPose() {
    return io.getPose();
  }

  public void resetPosition(Pose2d pose) {
    io.resetPosition(pose);
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return io.getRobotRelativeChassisSpeeds();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    io.setModuleStates(desiredStates);
  }

  public void stopModules() {
    io.stopModules();
  }

  public void drive(ChassisSpeeds speeds) {
    io.drive(speeds);
  }

  public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed) {
    io.driveFieldRelative(xSpeed, ySpeed, rotSpeed);
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    io.driveFieldRelative(speeds);
  }

  public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed) {
    io.driveRobotRelative(xSpeed, ySpeed, rotSpeed);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    io.driveRobotRelative(speeds);
  }


  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    this.io.periodic();
  }
}
