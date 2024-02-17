// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkMax wrist = new CANSparkMax(Constants.MotorConstants.wristMotorID, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(Constants.MotorConstants.intakeMotorID, MotorType.kBrushless);
  
  DigitalInput intakeSwitch = new DigitalInput(Constants.ManipulatorConstants.intakeLimitSwithPort);  
  
  private double wantedWristPosition = -1.0;
  private final double[] wPID = Constants.ManipulatorConstants.wristPID;
  public final PIDController wristPID = new PIDController(wPID[0], wPID[1], wPID[2]);

  /** Creates a new Intake. */
  public Intake() {
    intake.setInverted(Constants.MotorConstants.intakeMotorInversion);
    wrist.setInverted(Constants.MotorConstants.wristMotorInversion);

    intake.setIdleMode(IdleMode.kBrake);
    wrist.setIdleMode(IdleMode.kBrake);

    wrist.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double wristPos = wrist.getEncoder().getPosition();
    SmartDashboard.putNumber("Wrist Encoder", wristPos);

    if (wantedWristPosition != -1.0) {
      double power = wristPID.calculate(wristPos);
      SmartDashboard.putNumber("Wrist PID", power);
      setWristPower(MathUtil.clamp(power, -0.2, 0.2));
    }
  }

  public void setIntakePower(double intakePower) {
    intake.set(Constants.ManipulatorConstants.intakePower * intakePower);
  }

  public void setWristPower(double wristPower) {
    wantedWristPosition = -1.0;
    wrist.set(Constants.ManipulatorConstants.wristMaxPower * wristPower);
  }

  public void setWristSetPoint(double wristSetPoint) {
    wantedWristPosition = wristSetPoint;
    wristPID.setSetpoint(wantedWristPosition);
  }

  public boolean limitSwitch() {
    return false; //intakeSwitch.get();
  }
}
