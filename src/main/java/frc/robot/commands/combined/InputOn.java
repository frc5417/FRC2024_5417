// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeWristSetPoint;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InputOn extends SequentialCommandGroup {
  
  /** Creates a new InputOn. */
  public InputOn(Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.race(
        new IntakeWristSetPoint(intake, 27.8, true),
        new WaitCommand(0.5)
      ),
      Commands.race(
        new ToggleIntake(intake, -1, false, false),
        new WaitCommand(0.1)
      )
    );
  }
}
