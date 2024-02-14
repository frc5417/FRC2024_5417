// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  CANSparkMax shooter1 = new CANSparkMax(Constants.MotorConstants.shooter1MotorID, MotorType.kBrushless);
  CANSparkMax shooter2 = new CANSparkMax(Constants.MotorConstants.shooter2MotorID, MotorType.kBrushless);

  CANSparkMax intestine = new CANSparkMax(Constants.MotorConstants.intestineMotorID, MotorType.kBrushless);

  CANSparkMax pivot = new CANSparkMax(Constants.MotorConstants.pivotMotorID, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {
    shooter1.setInverted(Constants.MotorConstants.shooter1Inversion);
    shooter2.setInverted(Constants.MotorConstants.shooter2Inversion);
    intestine.setInverted(Constants.MotorConstants.intestineInversion);
    pivot.setInverted(Constants.MotorConstants.pivotInversion);

    shooter1.setIdleMode(IdleMode.kCoast);
    shooter2.setIdleMode(IdleMode.kCoast);
    intestine.setIdleMode(IdleMode.kBrake);
    pivot.setIdleMode(IdleMode.kBrake);

    pivot.getPIDController().setP(Constants.ManipulatorConstants.shooterPivotP);
    pivot.getPIDController().setI(Constants.ManipulatorConstants.shooterPivotI);
    pivot.getPIDController().setD(Constants.ManipulatorConstants.shooterPivotD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooter(double power) {
    shooter1.set(power);
    shooter2.set(power);
  }

  public void setPivotPosition(double pos) {
    
  }

  public void runIntestine() {
    intestine.set(Constants.ManipulatorConstants.intestinePower);
  }

  public void stopIntestine() {
    intestine.set(0);
  }
}
