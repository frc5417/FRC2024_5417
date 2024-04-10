// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CustomNamedCommands;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCurve extends SequentialCommandGroup {
  Pose2d startPose = new Pose2d(1.22, 5.55, Rotation2d.fromDegrees(0)); //right in front of speaker
  Pose2d pickUpNote1 = new Pose2d(2.33, 5.59, Rotation2d.fromDegrees(0)); //note in front of speaker
  Pose2d pickUpNote2 = new Pose2d(2.47, 6.72, Rotation2d.fromDegrees(0)); //note next to amp


  Pose2d[] forwardPath1 =  { startPose, pickUpNote1 };
  Pose2d[] backToSpeaker1 = { pickUpNote1, startPose };
  Pose2d[] forwardPath2 = { startPose, pickUpNote2 };
  Pose2d[] backToSpeaker2 = { pickUpNote2, startPose };

  /** Creates a new ShootForward. */
  public ShootCurve(DriveBase driveBase) {
    // Add your commands in the addCommands() call
    addCommands(
      CustomNamedCommands.getCommand("Shoot"),
      CustomNamedCommands.getCommand("IntakeIn"),
      new FollowBezier(driveBase, forwardPath1, 150, true),
      CustomNamedCommands.getCommand("PassOff"),
      new FollowBezier(driveBase, backToSpeaker1, 150, false),
      CustomNamedCommands.getCommand("Shoot"), // shoot 2nd note then go get 3rd
      CustomNamedCommands.getCommand("IntakeIn"),
      new FollowBezier(driveBase, forwardPath2, 150, false),
      CustomNamedCommands.getCommand("PassOff"),
      new FollowBezier(driveBase, backToSpeaker2, 150, false),
      CustomNamedCommands.getCommand("Shoot")
    );
  }
}
