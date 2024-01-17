// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeIO io;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setIntakePosition(double positionDeg) {
    this.io.setIntakePosition(positionDeg);
  };

  public Rotation2d getIntakePosition() {
    return this.io.getIntakePosition();
  };

  public void setRollerRpm(double speedRpm) {
    this.io.setRollerRpm(speedRpm);
  };

  public void setRollerSpeed(double speed) {
    this.io.setRollerSpeed(speed);
  };

  public double getRollerSpeed() {
    return this.io.getRollerSpeed();
  };

  public void stopRoller() {
    this.io.stopRoller();
  };

  public void stopIntake() {
    this.io.stopIntake();
  };


  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    this.io.periodic();
  }
}
