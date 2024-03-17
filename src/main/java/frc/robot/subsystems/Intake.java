// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  CANSparkMax wrist = new CANSparkMax(Constants.MotorConstants.wristMotorID, MotorType.kBrushless);
  TalonSRX intake = new TalonSRX(Constants.MotorConstants.intakeMotorID);
  
  DigitalInput intakeSwitch = new DigitalInput(Constants.ManipulatorConstants.intakeLimitSwithPort);  
  private int cyclesElapsed = 0;
  private double wantedWristPosition = 0.0;
  private final double[] wPID = Constants.ManipulatorConstants.intakeWristPID;
  public final PIDController wristPID = new PIDController(wPID[0], wPID[1], wPID[2]);
  // public final ArmFeedforward forwardPID = new ArmFeedforward(0, 0, 1.95);

  /** Creates a new Intake. */
  public Intake() {
    intake.setInverted(Constants.MotorConstants.intakeMotorInversion);
    wrist.setInverted(Constants.MotorConstants.wristMotorInversion);

    intake.setNeutralMode(NeutralMode.Brake);
    wrist.setIdleMode(IdleMode.kBrake);

    wrist.getEncoder().setPosition(0);
    wristPID.setTolerance(Constants.ManipulatorConstants.intakeWristTolerance);
    setWristSetPoint(wantedWristPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double wristPos = wrist.getEncoder().getPosition();
    SmartDashboard.putNumber("Wrist Encoder", wristPos);

    double power = wristPID.calculate(wristPos);
    SmartDashboard.putNumber("Wrist PID", power);
    
    if (!atWristSetPoint() && Robot.INSTANCE != null && Robot.INSTANCE.isEnabled()){
      cyclesElapsed ++;
    } else {
      cyclesElapsed = 0;
    }

    if (cyclesElapsed >= frc.robot.Constants.MotorConstants.maxWristPowerCycles){
      power = 0;
      if (cyclesElapsed % 20 == 0){ 
         System.out.println("WARNING: Intake pivot motor detected invalid setpoint. Disabling intake pivot.");
      }
    }

    setWristPower(MathUtil.clamp(power, -1, 1));

  }

  public void setIntakePower(double intakePower) {
    intake.set(TalonSRXControlMode.PercentOutput, Constants.ManipulatorConstants.intakePower * intakePower);
  }

  public void setWristPower(double wristPower) {
    wrist.set(Constants.ManipulatorConstants.intakeWristMaxPower * wristPower);
    SmartDashboard.putNumber("Wrist Power", Constants.ManipulatorConstants.intakeWristMaxPower * wristPower);
  }

  public void incrementWristPos(double wristSetPointDelta) {
    setWristSetPoint(wantedWristPosition + (wristSetPointDelta * Constants.ManipulatorConstants.intakeWristSetPointMaxDelta));
  }

  public void setWristSetPoint(double wristSetPoint) {
    wantedWristPosition = MathUtil.clamp(wristSetPoint, Constants.ManipulatorConstants.intakeWristMin, Constants.ManipulatorConstants.intakeWristMax);
    SmartDashboard.putNumber("Wrist SetPoint", wantedWristPosition);
    wristPID.setSetpoint(wantedWristPosition);
  }

  public boolean atWristSetPoint() {
    double wristPos = wrist.getEncoder().getPosition();
    return Math.abs(wristPos - wantedWristPosition) < 1;
  }

  public boolean limitSwitch() {
    return false; // intakeSwitch.get();
  }
}
