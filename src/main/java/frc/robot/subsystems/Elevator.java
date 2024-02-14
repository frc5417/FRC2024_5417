// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  CANSparkMax elevator = new CANSparkMax(Constants.MotorConstants.elevatorMotorID,MotorType.kBrushless);
  /** Creates a new Elevator. */
  public Elevator() {
    elevator.setInverted(Constants.MotorConstants.elevatorMotorInversion);

    elevator.setIdleMode(IdleMode.kBrake);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setElevatorPower(double elevatorPower) {
    elevator.set(Constants.ManipulatorConstants.elevatorMaxPower*elevatorPower);
  }
}
