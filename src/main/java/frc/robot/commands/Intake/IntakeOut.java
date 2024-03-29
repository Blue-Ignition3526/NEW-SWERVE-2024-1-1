// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.Constants;

public class IntakeOut extends Command {
  private Intake intake;

  /** Creates a new IntakeOut. */
  public IntakeOut(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setRollerRpm(Constants.Intake.kRollerOutSpeedRpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
