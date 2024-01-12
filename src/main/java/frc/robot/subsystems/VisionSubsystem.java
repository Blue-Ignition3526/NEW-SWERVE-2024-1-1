// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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

  Pose3d latestEstimatedPose;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    this.limeLight = new PhotonCamera(kLimelightName); // instantiate the camera

    // instantiate the pose estimator on multi-tag mode
    this.photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, limeLight, kRobotToLime);
    this.photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // set the fallback strategy to lowest ambiguity
  }

  /**
   * Get the latest estimated pose from the vision system
   * @return The latest pose of the robot in relation to the field
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = limeLight.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /**
   * Get the latest estimated pose from the vision system
   * @param AprilTagID the ID of the AprilTag you want to find
   * @return The latest pose of the AprilTag in relation to the robot
   * @throws Optional.empty if the AprilTag is not found
   */
  public Optional<Pose3d> getAprilTagPose(int AprilTagID) {
    if(!limeLight.getLatestResult().hasTargets()) return Optional.empty();

    List<PhotonTrackedTarget> targets = limeLight.getLatestResult().getTargets();
    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() == AprilTagID) {
        return Optional.of(new Pose3d(target.getAlternateCameraToTarget().getTranslation(), new Rotation3d(target.getSkew(), target.getPitch(), target.getYaw())));
      }
    }
    return Optional.empty();
  }

  public Optional<Double> calculateRotSpeed(int AprilTagID) {
    Optional<Pose3d> tagPose = getAprilTagPose(AprilTagID);
    if (!tagPose.isPresent()) {
      return Optional.empty();
    }
    PIDController rotPID = new PIDController(kActiveTrackPIDValues[0], kActiveTrackPIDValues[1], kActiveTrackPIDValues[2]);

    double rotSpeed = rotPID.calculate(tagPose.get().getX(), 0);
    
    rotPID.close();
    return Optional.of(rotSpeed);
  }

  /**
   * Get the latest estimated pose from the vision system
   * @return The latest pose of the robot in relation to the field
   */
  public Pose3d getLatestEstimatedPose() {
    return latestEstimatedPose;
  }
  
  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> pose = getEstimatedGlobalPose();
    if (pose.isPresent()) {
      Logger.recordOutput("VisionRobotPose", pose.get().estimatedPose);
      latestEstimatedPose = pose.get().estimatedPose;
    }
  }
}
