// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.RunIntestine;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Intestine extends ParallelCommandGroup {
  /** Creates a new PassOffPoint. */
  public Intestine(Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.

    addCommands(
      new RunIntestine(shooter, 1),
      new RunShooter(shooter, -0.1)
    );
  }
}
