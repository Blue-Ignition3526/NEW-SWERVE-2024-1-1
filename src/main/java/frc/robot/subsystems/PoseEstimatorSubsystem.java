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

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

public class PoseEstimatorSubsystem extends SubsystemBase { 

  SwerveDrive swerve;

  // https://www.chiefdelphi.com/t/multi-camera-setup-and-photonvisions-pose-estimator-seeking-advice/431154
  PhotonCamera limeLight;
  PhotonCamera frontCamera;
  PhotonCamera backCamera;
  PhotonCamera leftCamera;
  PhotonCamera rightCamera;

  PhotonPoseEstimator lime_photonEstimator;
  PhotonPoseEstimator front_photonEstimator;
  PhotonPoseEstimator back_photonEstimator;
  PhotonPoseEstimator left_photonEstimator;
  PhotonPoseEstimator right_photonEstimator;

  private double lime_lastEstTimestamp = 0;
  private double front_lastEstTimestamp = 0;
  private double back_lastEstTimestamp = 0;
  private double left_lastEstTimestamp = 0;
  private double right_lastEstTimestamp = 0;

  
  SwerveDrivePoseEstimator swerveEstimator;

  Pose3d latestEstimatedPose;

  private static int LIMELIGHT_CAMERA = 0;
  private static int FRONT_CAMERA = 1;
  private static int BACK_CAMERA = 2;
  private static int LEFT_CAMERA = 3;
  private static int RIGHT_CAMERA = 4;

  /** Creates a new VisionSubsystem. */
  public PoseEstimatorSubsystem(SwerveDrive swerve) {
    this.swerve = swerve;

    Logger.recordOutput("Vision/EnabledCameras/limeLight", kEnabledCameras[0]);
    Logger.recordOutput("Vision/EnabledCameras/front", kEnabledCameras[1]);
    Logger.recordOutput("Vision/EnabledCameras/back", kEnabledCameras[2]);
    Logger.recordOutput("Vision/EnabledCameras/left", kEnabledCameras[3]);
    Logger.recordOutput("Vision/EnabledCameras/right", kEnabledCameras[4]);

    this.limeLight = new PhotonCamera(kLimelightCameraName); // instantiate the camera
    
    this.frontCamera = new PhotonCamera(kFrontCameraName); // instantiate the camera
    this.backCamera = new PhotonCamera(kBackCameraName); // instantiate the camera
    this.leftCamera = new PhotonCamera(kLeftCameraName); // instantiate the camera
    this.rightCamera = new PhotonCamera(kRightCameraName); // instantiate the camera

    // instantiate the pose estimator on multi-tag mode
    this.lime_photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, limeLight, kRobotToLime);
    this.lime_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // set the fallback strategy to lowest ambiguity

    this.front_photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera, kRobotToFrontCamera);
    this.front_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // set the fallback strategy to lowest ambiguity

    this.back_photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera, kRobotToBackCamera);
    this.back_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // set the fallback strategy to lowest ambiguity

    this.left_photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCamera, kRobotToLeftCamera);
    this.left_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // set the fallback strategy to lowest ambiguity

    this.right_photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCamera, kRobotToRightCamera);
    this.right_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // set the fallback strategy to lowest ambiguity

    this.swerveEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.Physical.m_swerveDriveKinematics, this.swerve.getRotation2d(), swerve.getModulePositions(), Constants.Field.kStartPose, Constants.Swerve.PoseEstimation.kStateStdDevs, kMultiTagStdDevs);
  }

  /**
   * Get the latest estimated pose from the vision system
   * @param camera the camera you want to get the pose from
   * @return The latest pose of the robot in relation to the field
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(int camera) {
    if (camera == LIMELIGHT_CAMERA) {
      var visionEst = lime_photonEstimator.update();
      double latestTimestamp = limeLight.getLatestResult().getTimestampSeconds();

      // Logger.recordOutput("Vision/LimeLight/BestTargetAmbiguity",
      //     limeLight.getLatestResult().getBestTarget().getPoseAmbiguity());

      boolean newResult = Math.abs(latestTimestamp - lime_lastEstTimestamp) > 1e-5;

      if (newResult) lime_lastEstTimestamp = latestTimestamp;

      return visionEst;

    } else if (camera == FRONT_CAMERA) {
      var visionEst = front_photonEstimator.update();
      double latestTimestamp = frontCamera.getLatestResult().getTimestampSeconds();

      Logger.recordOutput("Vision/Front/BestTargetAmbiguity",
          frontCamera.getLatestResult().getBestTarget().getPoseAmbiguity());

      boolean newResult = Math.abs(latestTimestamp - front_lastEstTimestamp) > 1e-5;

      if (newResult) front_lastEstTimestamp = latestTimestamp;

      return visionEst;

    } else if (camera == BACK_CAMERA) {
      var visionEst = back_photonEstimator.update();
      double latestTimestamp = backCamera.getLatestResult().getTimestampSeconds();

      Logger.recordOutput("Vision/Back/BestTargetAmbiguity",
          backCamera.getLatestResult().getBestTarget().getPoseAmbiguity());

      boolean newResult = Math.abs(latestTimestamp - back_lastEstTimestamp) > 1e-5;

      if (newResult) back_lastEstTimestamp = latestTimestamp;

      return visionEst;

    } else if (camera == LEFT_CAMERA) {
      var visionEst = left_photonEstimator.update();
      double latestTimestamp = leftCamera.getLatestResult().getTimestampSeconds();
      Logger.recordOutput("Vision/Left/BestTargetAmbiguity",
          leftCamera.getLatestResult().getBestTarget().getPoseAmbiguity());

      boolean newResult = Math.abs(latestTimestamp - left_lastEstTimestamp) > 1e-5;

      if (newResult) left_lastEstTimestamp = latestTimestamp;

      return visionEst;

    } else if (camera == RIGHT_CAMERA) {
      var visionEst = right_photonEstimator.update();
      double latestTimestamp = rightCamera.getLatestResult().getTimestampSeconds();

      Logger.recordOutput("Vision/Right/BestTargetAmbiguity",
          rightCamera.getLatestResult().getBestTarget().getPoseAmbiguity());

      boolean newResult = Math.abs(latestTimestamp - right_lastEstTimestamp) > 1e-5;

      if (newResult) right_lastEstTimestamp = latestTimestamp;

      return visionEst;
      
    } else {
      return Optional.empty();
    }
  }

  /**
   * Get the latest estimated pose from the vision system from all cameras
   * @return an array of the latest poses {LimeLight, Front, Back, Left, Right}
   * @throws Optional.empty if the AprilTag is not found
   */
  // public Optional<EstimatedRobotPose>[] getAllEstiamtedPoses() {
  //   Optional<EstimatedRobotPose>[] allPoses = new Optional[5];
  //   allPoses[0] = kEnabledCameras[0] && getEstimatedGlobalPose(LIMELIGHT_CAMERA).isPresent() ? getEstimatedGlobalPose(LIMELIGHT_CAMERA) : Optional.empty();
  //   // allPoses[1] = kEnabledCameras[1] && getEstimatedGlobalPose(FRONT_CAMERA).isPresent() ? getEstimatedGlobalPose(FRONT_CAMERA) : Optional.empty();
  //   // allPoses[2] = kEnabledCameras[2] && getEstimatedGlobalPose(BACK_CAMERA).isPresent() ? getEstimatedGlobalPose(BACK_CAMERA) : Optional.empty();
  //   // allPoses[3] = kEnabledCameras[3] && getEstimatedGlobalPose(LEFT_CAMERA).isPresent() ? getEstimatedGlobalPose(LEFT_CAMERA) : Optional.empty();
  //   // allPoses[4] = kEnabledCameras[4] && getEstimatedGlobalPose(RIGHT_CAMERA).isPresent() ? getEstimatedGlobalPose(RIGHT_CAMERA) : Optional.empty();
  //   return allPoses;
  // }

  /**
   * Get the latest estimated pose from the vision system
   * @param AprilTagID the ID of the AprilTag you want to find
   * @return The latest pose of the AprilTag in relation to the robot
   * @throws Optional.empty if the AprilTag is not found
   */
  public Optional<Double> getAprilTagOffset(int AprilTagID) {
    if(!limeLight.getLatestResult().hasTargets() && !rightCamera.getLatestResult().hasTargets()) return Optional.empty();

    List<PhotonTrackedTarget> targets = limeLight.getLatestResult().getTargets();
    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() == AprilTagID) {
        double tagOffset = target.getYaw();
        Logger.recordOutput("Vision/TagOffset", tagOffset);
        return Optional.of(tagOffset);
      }
    }

    targets = rightCamera.getLatestResult().getTargets();
    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() == AprilTagID) {
        double tagOffset = target.getYaw() + 35; //! change 30 for better number
        Logger.recordOutput("Vision/TagOffset", tagOffset);
        return Optional.of(tagOffset);
      }
    }
    return Optional.empty();
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
    // Optional<EstimatedRobotPose>[] allPoses = getAllEstiamtedPoses();
    
    // if(kEnabledCameras[0] && allPoses[0].isPresent()) Logger.recordOutput("Vision/LimeLight/estimatedPose", allPoses[0].get().estimatedPose);
    // if(kEnabledCameras[1]) Logger.recordOutput("Vision/Front/estimatedPose", allPoses[1].get().estimatedPose);
    // if(kEnabledCameras[2]) Logger.recordOutput("Vision/Back/estimatedPose", allPoses[2].get().estimatedPose);
    // if(kEnabledCameras[3]) Logger.recordOutput("Vision/Left/estimatedPose", allPoses[3].get().estimatedPose);
    // if(kEnabledCameras[4]) Logger.recordOutput("Vision/Right/estimatedPose", allPoses[4].get().estimatedPose);

    // if(kEnabledCameras[0]) swerveEstimator.addVisionMeasurement(allPoses[0].get().estimatedPose.toPose2d(), lime_lastEstTimestamp); // add the latest limeLight pose to the estimator
    // if(kEnabledCameras[1]) swerveEstimator.addVisionMeasurement(allPoses[1].get().estimatedPose.toPose2d(), front_lastEstTimestamp); // add the latest front pose to the estimator
    // if(kEnabledCameras[2]) swerveEstimator.addVisionMeasurement(allPoses[2].get().estimatedPose.toPose2d(), back_lastEstTimestamp); // add the latest back pose to the estimator
    // if(kEnabledCameras[3]) swerveEstimator.addVisionMeasurement(allPoses[3].get().estimatedPose.toPose2d(), left_lastEstTimestamp); // add the latest left pose to the estimator
    // if(kEnabledCameras[4]) swerveEstimator.addVisionMeasurement(allPoses[4].get().estimatedPose.toPose2d(), right_lastEstTimestamp); // add the latest right pose to the estimator

    // swerveEstimator.update(swerve.getRotation2d(), swerve.getModulePositions()); // update the estimator

    Logger.recordOutput("Vision/SwervePoseEstimator/EstimatedPose", swerveEstimator.getEstimatedPosition()); // Log pose in periodic

    if(getAprilTagOffset(AprilTags.kSpeakerTagID).isPresent()) Logger.recordOutput("Vision/LimeLight/TagIsVisible", true);
      else Logger.recordOutput("Vision/LimeLight/TagIsVisible", false); // Log pose in periodic
  }
}