// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveBase m_driveBase;

  double prev_omega = 0;
  double prev_xVel = 0;
  double prev_yVel = 0;
 
  int counter = 0;

  double manipulatorPosition = 0;

  public TeleopDrive(DriveBase driveBase) {
    m_driveBase = driveBase;
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = (RobotContainer.getDriverLeftJoyX() * 0.45) + (prev_xVel * 0.55); 
    double yVel = (RobotContainer.getDriverLeftJoyY() * 0.45) + (prev_yVel * 0.55); 
    double omega = (RobotContainer.getDriverRightJoyX() * 0.225) + (prev_omega * 0.55);

    prev_xVel = xVel;
    prev_yVel = yVel;
    prev_omega = omega;

    SmartDashboard.putNumber("X-Vel Input", xVel);
    SmartDashboard.putNumber("Y-Vel Input", yVel);
    SmartDashboard.putNumber("Omega Vel Input", omega);
    
    m_driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(xVel, yVel, omega));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBase.resetDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
