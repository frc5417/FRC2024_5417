// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CustomNamedCommands;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedAmpMobile extends SequentialCommandGroup {
  Pose2d startPose = new Pose2d(15.80, 6.51, Rotation2d.fromDegrees(120)); //right in front of speaker
  Pose2d opponentsarebozos = new Pose2d(12.65, 7.86, Rotation2d.fromDegrees(120)); //note in front of speaker

  // Pose2d bookit = new Pose2d(9.6, 0.75, Rotation2d.fromDegrees(240));

  Pose2d[] forwardPath =  { startPose, opponentsarebozos };
  // Pose2d[] bookitPath = { opponentsarebozos, bookit };
  
  // Pose2d[] backToSpeaker = { pickUpNote1, startPose };

  /** Creates a new ShootForward. */
  public RedAmpMobile(DriveBase driveBase) {
    // Add your commands in the addCommands() call
    addCommands(
      CustomNamedCommands.getCommand("Shoot"),
      new WaitCommand(10.0),
      new FollowBezier(driveBase, forwardPath, 10, true)
    );
  }
}
