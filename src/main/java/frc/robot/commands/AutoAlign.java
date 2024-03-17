// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AutoAlign extends Command {
  private int cantFindCount = 0;
  private final DriveBase driveBase;
  private final Shooter shooter;

  private final PIDConstants rotationPID = Constants.DriveTrainConstants.ROTATION_PID;
  private final PIDController drivePID = new PIDController(rotationPID.kP, rotationPID.kI, rotationPID.kD);

  /** Creates a new AutoAlign. */
  public AutoAlign(DriveBase driveBase, Shooter shooter) {
    this.driveBase = driveBase;
    this.shooter = shooter;

    drivePID.setTolerance(0.15);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cantFindCount = 0;

    System.out.println("AutoAligned started.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Vision.getTID() <= 0) {
      cantFindCount += 1;
      return;
    }

    double currentAngle = driveBase.getCurrentPose().getRotation().getRadians();
    double wantedAngle = currentAngle + Vision.getAdjustedHorizontalAngle();

    SmartDashboard.putNumber("currentAngleAuto", currentAngle);
    SmartDashboard.putNumber("wantedAngleAuto", wantedAngle);

    drivePID.setSetpoint(wantedAngle);
    double omega = MathUtil.clamp(drivePID.calculate(currentAngle), -1, 1);

    // driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(0, 0, omega));

    shooter.goToTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AutoAligned ended.");
    cantFindCount = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cantFindCount >= 40 || (shooter.atWristSetPoint() && drivePID.atSetpoint());
  }
}
