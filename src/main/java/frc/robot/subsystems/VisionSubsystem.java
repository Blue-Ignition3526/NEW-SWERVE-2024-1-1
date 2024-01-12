// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import static frc.robot.Constants.Vision.*;
import java.util.Optional;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase { 
  // TODO: add multiple cameras
  // https://www.chiefdelphi.com/t/multi-camera-setup-and-photonvisions-pose-estimator-seeking-advice/431154
  PhotonCamera limeLight;
  
  PhotonCamera frontCamera;
  PhotonCamera backCamera;
  PhotonCamera leftCamera;
  PhotonCamera rightCamera;

  PhotonPoseEstimator photonEstimator;
  private double lastEstTimestamp = 0;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    this.limeLight = new PhotonCamera(kLimelightCameraName); // instantiate the camera
    
    this.frontCamera = new PhotonCamera(kFrontCameraName); // instantiate the camera
    this.backCamera = new PhotonCamera(kBackCameraName); // instantiate the camera
    this.leftCamera = new PhotonCamera(kLeftCameraName); // instantiate the camera
    this.rightCamera = new PhotonCamera(kRightCameraName); // instantiate the camera

    // instantiate the pose estimator on multi-tag mode
    this.photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, limeLight, kRobotToLime);
    this.photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // set the fallback strategy to lowest ambiguity
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = limeLight.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }
  
  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> pose = getEstimatedGlobalPose();
    if (pose.isPresent()) {
      Logger.recordOutput("VisionRobotPose", pose.get().estimatedPose);
    }
  }
}
