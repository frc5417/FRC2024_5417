// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CustomNamedCommands;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedRightTwo extends SequentialCommandGroup {
  Pose2d startPose = new Pose2d(15.96, 6.53, Rotation2d.fromDegrees(120)); //right in front of speaker
  Pose2d note1 = new Pose2d(14, 7, Rotation2d.fromDegrees(180)); //left of speaker

  Pose2d[] forwardPath =  { startPose, note1 };
  Pose2d[] backToSpeaker = { note1, startPose };

  /** Creates a new ShootForward. */
  public RedRightTwo(DriveBase driveBase) {
    // Add your commands in the addCommands() call
    addCommands(
      CustomNamedCommands.getCommand("Shoot"),
      Commands.parallel(
        CustomNamedCommands.getCommand("IntakeIn"),
        new FollowBezier(driveBase, forwardPath, 55, true).withTimeout(3)
      ),
      Commands.parallel(
        CustomNamedCommands.getCommand("PassOff"),
        new FollowBezier(driveBase, backToSpeaker, 55, false).withTimeout(3)
      ),
      CustomNamedCommands.getCommand("Shoot")
    );
  }
}
