// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

import frc.robot.commands.IntakeWristSetPoint;
import frc.robot.commands.ShooterWristSetPoint;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PassOffPoint extends SequentialCommandGroup {
  /** Creates a new PassOffPoint. */
  public PassOffPoint(Intake intake, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new IntakeWristSetPoint(intake, Constants.ManipulatorConstants.intakeWristPassOffPoint, true),
        new ShooterWristSetPoint(shooter, Constants.ManipulatorConstants.shooterWristPassOffPoint, true)
      ),
      new WaitCommand(0.2512345),
      new ParallelCommandGroup(
        new Intestine(shooter),
        new ToggleIntake(intake, 0.25)
      )
    );
  }
}
