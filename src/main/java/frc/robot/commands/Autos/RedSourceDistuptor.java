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
public class RedSourceDistuptor extends SequentialCommandGroup {
  Pose2d startPose = new Pose2d(16.00, 4.5, Rotation2d.fromDegrees(240)); //right in front of speaker
  Pose2d middlerandopoint = new Pose2d(13.63, 1.25, Rotation2d.fromDegrees(240));
  Pose2d opponentsarebozos = new Pose2d(8.6, 0.75, Rotation2d.fromDegrees(240)); //note in front of speaker

  Pose2d bookit = new Pose2d(9.6, 0.75, Rotation2d.fromDegrees(240));

  Pose2d[] forwardPath =  { startPose, middlerandopoint, opponentsarebozos };
  Pose2d[] bookitPath = { opponentsarebozos, bookit };
  
  // Pose2d[] backToSpeaker = { pickUpNote1, startPose };

  /** Creates a new ShootForward. */
  public RedSourceDistuptor(DriveBase driveBase) {
    // Add your commands in the addCommands() call
    addCommands(
      CustomNamedCommands.getCommand("Shoot"),
      new WaitCommand(2.0),
      new FollowBezier(driveBase, forwardPath, 200, true),
      new FollowBezier(driveBase, bookitPath, 200, false)
    );
  }
}
