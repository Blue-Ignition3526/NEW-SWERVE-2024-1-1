// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

public class PoseEstimatorSubsystem extends SubsystemBase { 

  SwerveDrive swerve;
  
  SwerveDrivePoseEstimator swerveEstimator;

  Pose3d latestEstimatedPose;

  /** Creates a new VisionSubsystem. */
  public PoseEstimatorSubsystem(SwerveDrive swerve) {
    this.swerve = swerve;

    this.swerveEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.Physical.m_swerveDriveKinematics, this.swerve.getRotation2d(), swerve.getModulePositions(), Constants.Field.kStartPose, Constants.Swerve.PoseEstimation.kStateStdDevs, kMultiTagStdDevs);
  }
  
  public Pose3d getLatestEstimatedPose() {
    return latestEstimatedPose;
  }

  @Override
  public void periodic() {
    swerveEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d(kLimelightName), LimelightHelpers.getLatency_Pipeline(kLimelightName)); // add vision measurement to the estimator

    swerveEstimator.update(swerve.getRotation2d(), swerve.getModulePositions()); // update the estimator

    Logger.recordOutput("Vision/SwervePoseEstimator/EstimatedPose", swerveEstimator.getEstimatedPosition()); // Log pose in periodic
  }
}
