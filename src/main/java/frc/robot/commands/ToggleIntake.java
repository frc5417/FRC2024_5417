// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends Command {
  private final Intake intake;
  private final boolean doDisable;
  private final boolean stopOnDisable;
  private final double initialDirection;
  private double direction;

  private final double seconds = 0.6;
  private int counter = 0;

  public ToggleIntake(Intake intake, double direction) {
    this(intake, direction, true, true);
  }

  public ToggleIntake(Intake intake, double direction, boolean doDisable) {
    this(intake, direction, doDisable, true);
  }

  /** Creates a new ToggleIntake. */
  public ToggleIntake(Intake intake, double direction, boolean doDisable, boolean stopOnDisable) {
    this.intake = intake;
    this.initialDirection = direction;
    this.direction = direction;
    this.doDisable = doDisable;
    this.stopOnDisable = stopOnDisable;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    direction = initialDirection;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(RobotContainer.getDPadUp()){
    //   intake.setIntakePower(0.45);
    // }else{
      intake.setIntakePower(direction);
    // }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stopOnDisable) {
      intake.setIntakePower(0);
    }
    counter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!intake.limitSwitch() && direction < 0) {
      counter++;
      if (counter < (50 * seconds)) {
        direction = 0;
        return doDisable;
      }
    }
    return false;
  }
}
