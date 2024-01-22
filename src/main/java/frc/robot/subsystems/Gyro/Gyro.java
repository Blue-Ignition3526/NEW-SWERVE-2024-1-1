// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Gyro;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  GyroIO io;
  GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

  /** Creates a new Gyro. */
  public Gyro(GyroIO io) {
    this.io = io;
  }

  public double getPitch() {
    return io.getPitch();
  }

  public double getYaw() {
    return io.getYaw();
  }

  public double getRoll() {
    return io.getRoll();
  }

  public double getPitchVelocity() {
    return io.getPitchVelocity();
  }

  public double getYawVelocity() {
    return io.getYawVelocity();
  }

  public double getRollVelocity() {
    return io.getRollVelocity();
  }

  public double getAccelerationX() {
    return io.getAccelerationX();
  }

  public double getAccelerationY() {
    return io.getAccelerationY();
  }

  public double getAccelerationZ() {
    return io.getAccelerationZ();
  }

  public void reset() {
    io.reset();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
  }
}
