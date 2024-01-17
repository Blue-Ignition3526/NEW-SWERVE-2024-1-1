// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private SwerveModuleIO io;
  private SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

  /** Creates a new SwerveModule. */
  public SwerveModule(SwerveModuleIO io) {
    this.io = io;
  }

  public void stop() {
    io.stop();
  }

  public void setState(SwerveModuleState state) {
    io.setState(state);
  }

  public String getName() {
    return io.getName();
  }

  public SwerveModuleState getState() {
    return io.getState();
  }

  public SwerveModulePosition getPosition() {
    return io.getPosition();
  }


  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    this.io.periodic();
  }
}
