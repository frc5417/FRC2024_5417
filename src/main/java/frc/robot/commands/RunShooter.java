// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  private final Shooter shooter;
  private final double power;

  /** Creates a new RunShooter. */
  public RunShooter(Shooter shooter, double power) {
    this.shooter = shooter;
    this.power = power;

    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(shooter);
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooter(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
