// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterWristJoystick extends Command {
  /** Creates a new ElevatorJoystick. */
  private final Shooter shooter;

  public ShooterWristJoystick(Shooter shooter) {
    this.shooter = shooter;
    
    // Use addRequirements() here to declare subsystem dependencies.
    // Commented because it is constantly running - commands that always run should not stop other commands from running.
    // addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.incrementWristPos(RobotContainer.getManipulatorLeftJoyY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setWristPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
