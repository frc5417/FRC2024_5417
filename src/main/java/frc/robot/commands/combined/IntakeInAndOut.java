// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeWristSetPoint;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeInAndOut extends SequentialCommandGroup {
  
  /** Creates a new InputOn. */
  public IntakeInAndOut(Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeWristSetPoint(intake, 27.8, true).withTimeout(0.5),
      new ToggleIntake(intake, -1, false, false).withTimeout(0.1)
    );
  }
}
