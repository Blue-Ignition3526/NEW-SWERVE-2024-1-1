// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Vision.kLimelightCameraName;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightPose extends SubsystemBase {
  private NetworkTable table;

  public boolean isConnected = false;

  private boolean targetVisible;
  private double targetX;
  private double targetY;
  private int targetID;

  class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {
            resetPilelineLatency();
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
            if(getPipelineLatency()==0.0){
                isConnected = false;
            }else{
                isConnected = true;
            }
        }
    }
    Notifier _hearBeat = new Notifier(new PeriodicRunnable());

    /**
     * tl The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
     * @return
     */
    public double getPipelineLatency() {
        NetworkTableEntry tl = table.getEntry("tl");
        double l = tl.getDouble(0.0);
        return l;
    }

    private void resetPilelineLatency(){
        table.getEntry("tl").setValue(0.0);
    }


  /** Creates a new LimeLightPose. */
  public LimeLightPose() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    _hearBeat.startPeriodic(0.1);
  }

  /**
   * Get the offset of the closest AprilTag
   * @return The latest pose of the AprilTag in relation to the robot
   * @throws Optional.empty if the AprilTag is not found
   */
  public Optional<Double> getClosestAprilTagOffset() {
    if (targetVisible) {
      return Optional.of(targetX);
    } else {
      return Optional.empty();
    }
  }

  /**
   * Get the fiducial ID of the closest AprilTag found
   * @return The latest pose of the AprilTag in relation to the robot
   * @throws Optional.empty if the AprilTag is not found
   */
  public Optional<Integer> getClosestAprilTagID() {
    if (targetVisible) {
      return Optional.of(targetID);
    } else {
      return Optional.empty();
    }
  }

  /**
   * Get the X position of the target
   */
  public double getTargetX(){
    return targetX;
  }

  /**
   * Get the Y position of the target
   */
  public double getTargetY(){
    return targetY;
  }

  /**
   * Get the ID of the target
   */
  public boolean getTargetVisible(){
    return targetVisible;
  }

  @Override
  public void periodic() {
    targetVisible = table.getEntry("tv").getDouble(0.0) == 1.0;
    targetX = table.getEntry("tx").getDouble(0.0);
    targetY = table.getEntry("ty").getDouble(0.0);
    targetID = (int) table.getEntry("tid").getDouble(0.0);

    Logger.recordOutput("LimeLight/isConected", isConnected);

    Logger.recordOutput("LimeLight/tv", targetVisible);
    Logger.recordOutput("LimeLight/tx", targetX);
    Logger.recordOutput("LimeLight/ty", targetY);
    Logger.recordOutput("LimeLight/tid", targetID);
  }
}
