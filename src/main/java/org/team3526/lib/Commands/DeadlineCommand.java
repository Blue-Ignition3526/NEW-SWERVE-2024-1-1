// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team3526.lib.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Runs a command with a time deadline
 */
public class DeadlineCommand extends Command {
  Timer timer;
  Command command;
  double deadline;

  /** Creates a new DeadlineCommand. */
  public DeadlineCommand(Command command, double deadline) {
    this.command = command;
    this.deadline = deadline;
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= deadline || command.isFinished();
  }
}
