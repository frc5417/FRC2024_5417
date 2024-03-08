// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  TalonSRX elevator = new TalonSRX(Constants.MotorConstants.elevatorMotorID);
  /** Creates a new Elevator. */
  public Elevator() {
    elevator.setInverted(Constants.MotorConstants.elevatorMotorInversion);
    elevator.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElevatorPower(double elevatorPower) {
    SmartDashboard.putNumber("Elevator Power",  Constants.ManipulatorConstants.elevatorMaxPower * elevatorPower);

    elevator.set(TalonSRXControlMode.PercentOutput, Constants.ManipulatorConstants.elevatorMaxPower * elevatorPower);
  }
}
