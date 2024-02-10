// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkMax wrist = new CANSparkMax(Constants.MotorConstants.wristMotorID,MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(Constants.MotorConstants.intakeMotorID,MotorType.kBrushless);
  DigitalInput intakeSwitch = new DigitalInput(Constants.ManipulatorConstants.limitSwithPort);  
  /** Creates a new Intake. */
  public Intake() {
    intake.setInverted(Constants.MotorConstants.intakeMotorInversion);
    wrist.setInverted(Constants.MotorConstants.wristMotorInversion);

    intake.setIdleMode(IdleMode.kBrake);
    wrist.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakePower(double intakePower) {
    intake.set(Constants.ManipulatorConstants.intakePower * intakePower);
  }
  public boolean limitSwitch() {
    return intakeSwitch.get();
  }
}
