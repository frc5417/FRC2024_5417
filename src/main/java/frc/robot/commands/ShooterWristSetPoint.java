// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterWristSetPoint extends Command {
  /** Creates a new ElevatorJoystick. */
  private final Shooter shooter;
  private final double setPoint;
  private final boolean waitUntil;

  public ShooterWristSetPoint(Shooter shooter, double setPoint) {
    this(shooter, setPoint, false);
  }

  public ShooterWristSetPoint(Shooter shooter, double setPoint, boolean waitUntil) {
    this.shooter = shooter;
    this.setPoint = setPoint;
    this.waitUntil = waitUntil;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setWristSetPoint(this.setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !waitUntil || shooter.atWristSetPoint();
  }
}
