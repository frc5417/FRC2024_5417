// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AutoAlign extends Command {
  private final DriveBase driveBase;
  private final Shooter shooter;

  private final PIDConstants rotationPID = Constants.DriveTrainConstants.ROTATION_PID;
  private final PIDController drivePID = new PIDController(rotationPID.kP, rotationPID.kI, rotationPID.kD);

  /** Creates a new AutoAlign. */
  public AutoAlign(DriveBase driveBase, Shooter shooter) {
    this.driveBase = driveBase;
    this.shooter = shooter;

    drivePID.setTolerance(0.2);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Vision.getTID() <= 0) return;

    double currentAngle = driveBase.getCurrentPose().getRotation().getRadians();
    double wantedAngle = Math.toRadians(Vision.getTX());
    drivePID.setSetpoint(wantedAngle);
    double omega = MathUtil.clamp(drivePID.calculate(currentAngle), -1, 1);

    driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(0, 0, omega));

    shooter.goToTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.atWristSetPoint() && drivePID.atSetpoint();
  }
}
