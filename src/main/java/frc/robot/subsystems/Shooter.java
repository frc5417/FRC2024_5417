// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  CANSparkMax shooter1 = new CANSparkMax(Constants.MotorConstants.shooter1MotorID, MotorType.kBrushless);
  CANSparkMax shooter2 = new CANSparkMax(Constants.MotorConstants.shooter2MotorID, MotorType.kBrushless);

  CANSparkMax intestine = new CANSparkMax(Constants.MotorConstants.intestineMotorID, MotorType.kBrushless);

  CANSparkMax wrist = new CANSparkMax(Constants.MotorConstants.pivotMotorID, MotorType.kBrushless);

  private double wantedWristPosition = 0.0;
  private final double[] wPID = Constants.ManipulatorConstants.shooterWristPID;
  public final PIDController wristPID = new PIDController(wPID[0], wPID[1], wPID[2]);
  // public final ArmFeedforward forwardPID = new ArmFeedforward(0, 0, 1.95);

  /** Creates a new Shooter. */
  public Shooter() {
    shooter1.setInverted(Constants.MotorConstants.shooter1Inversion);
    shooter2.setInverted(Constants.MotorConstants.shooter2Inversion);
    intestine.setInverted(Constants.MotorConstants.intestineInversion);
    wrist.setInverted(Constants.MotorConstants.pivotInversion);

    shooter1.setIdleMode(IdleMode.kCoast);
    shooter2.setIdleMode(IdleMode.kCoast);
    intestine.setIdleMode(IdleMode.kBrake);
    wrist.setIdleMode(IdleMode.kBrake);

    wristPID.setTolerance(Constants.ManipulatorConstants.shooterWristTolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double wristPos = wrist.getEncoder().getPosition();
    SmartDashboard.putNumber("Shooter Wrist Encoder", wristPos);

    double power = wristPID.calculate(wristPos);
    SmartDashboard.putNumber("Shooter Wrist PID", power);
    setWristPower(MathUtil.clamp(power, -1, 1));
  }

  public void setShooter(double power) {
    shooter1.set(power);
    shooter2.set(power);
  }

  public void setWristPower(double wristPower) {
    wrist.set(Constants.ManipulatorConstants.shooterWristMaxPower * wristPower);
    SmartDashboard.putNumber("Shooter Wrist Power", Constants.ManipulatorConstants.intakeWristMaxPower * wristPower);
  }

  public void incrementWristPos(double wristSetPointDelta) {
    setWristSetPoint(wantedWristPosition + (wristSetPointDelta * Constants.ManipulatorConstants.shooterWristSetPointMaxDelta));
  }

  public void setWristSetPoint(double wristSetPoint) {
    wantedWristPosition = MathUtil.clamp(wristSetPoint, Constants.ManipulatorConstants.shooterWristMin, Constants.ManipulatorConstants.shooterWristMax);
    wristPID.setSetpoint(wantedWristPosition);
  }

  public void runIntestine() {
    intestine.set(Constants.ManipulatorConstants.intestinePower);
  }

  public void stopIntestine() {
    intestine.set(0);
  }
}
