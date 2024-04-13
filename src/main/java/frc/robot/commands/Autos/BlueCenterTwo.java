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
public class BlueCenterTwo extends SequentialCommandGroup {
  Pose2d startPose = new Pose2d(1.22, 5.55, Rotation2d.fromDegrees(0)); //right in front of speaker
  Pose2d pickUpNote1 = new Pose2d(2.33, 5.55, Rotation2d.fromDegrees(0)); //note in front of speaker

  Pose2d[] forwardPath =  { startPose, pickUpNote1 };
  Pose2d[] backToSpeaker = { pickUpNote1, startPose };

  /** Creates a new ShootForward. */
  public BlueCenterTwo(DriveBase driveBase) {
    // Add your commands in the addCommands() call
    addCommands(
      CustomNamedCommands.getCommand("Shoot"),
      Commands.parallel(
        CustomNamedCommands.getCommand("IntakeIn"),
        new FollowBezier(driveBase, forwardPath, 55, true)
      ),
      Commands.parallel(
        CustomNamedCommands.getCommand("PassOff"),
        new FollowBezier(driveBase, backToSpeaker, 55, false)
      ),
      CustomNamedCommands.getCommand("Shoot")
    );
  }
}
