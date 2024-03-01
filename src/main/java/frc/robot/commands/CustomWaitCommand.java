// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class CustomWaitCommand extends Command {
  private int runsLeft = 0;

  /** Creates a new CustomWaitCommand. */
  public CustomWaitCommand(double seconds) {
    runsLeft = (int) (seconds * (1000.0 / 20.0));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runsLeft -= 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runsLeft = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return runsLeft <= 0;
  }
}
