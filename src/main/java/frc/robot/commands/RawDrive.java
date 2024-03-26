// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RawDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveBase m_driveBase;

  private final PIDConstants rotationPID = Constants.DriveTrainConstants.ROTATION_PID;
  private final PIDController drivePID = new PIDController(rotationPID.kP, rotationPID.kI, rotationPID.kD);

  double x = 0;
  double y = 0;
  double omeg = 0;
  double startingAngle = 0;
  int startingCount = 0;
  int count = 0;

  double prev_omega = 0;
  double prev_xVel = 0;
  double prev_yVel = 0;
 
  public RawDrive(DriveBase driveBase, double x, double y, double omega, int count) {
    m_driveBase = driveBase;

    this.x = x;
    this.y = y;
    this.omeg = omega;
    this.startingCount = count;
    this.count = count;

    drivePID.setTolerance(0.15);
    if (omega == 0) {
      drivePID.setSetpoint(startingAngle);
    }
  }

  @Override
  public void initialize() {
    count = startingCount;
    this.startingAngle = m_driveBase.getCurrentPose().getRotation().getRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = (x * 0.125) + (prev_xVel * 0.875); 
    double yVel = (y * 0.125) + (prev_yVel * 0.875); 
    double omega = (omeg * 0.225) + (prev_omega * 0.55);

    // if (omeg == 0) {
    //   double currentAngle = m_driveBase.getCurrentPose().getRotation().getRadians();
    //   drivePID.setSetpoint(startingAngle);

    //   SmartDashboard.putNumber("currentAngleAuto", currentAngle);
    //   SmartDashboard.putNumber("wantedAngleAuto", startingAngle);

    //   omega = MathUtil.clamp(drivePID.calculate(currentAngle), -1, 1);
    //   omega *= 0.5;
    // }

    prev_xVel = xVel;
    prev_yVel = yVel;
    prev_omega = omega;

    count -= 1;
    
    m_driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(xVel, yVel, omega));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count <= 0;
  }
  
}
