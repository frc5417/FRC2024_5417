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
public class RedCenterThree extends SequentialCommandGroup {
  Pose2d startPose = new Pose2d(15.33, 5.55, Rotation2d.fromDegrees(180)); //right in front of speaker
  Pose2d closeShoot = new Pose2d(14.93, 5.55, Rotation2d.fromDegrees(180));
  Pose2d note1 = new Pose2d(14, 5.55, Rotation2d.fromDegrees(180)); // note in front of speaker
  Pose2d left = new Pose2d(14.93, 4.12, Rotation2d.fromDegrees(180));
  Pose2d note2 = new Pose2d(14, 4.12, Rotation2d.fromDegrees(180)); //left of speaker

  Pose2d[] forwardPath1 =  { startPose, note1 };
  Pose2d[] backToSpeaker1 = { note1, closeShoot };
  Pose2d[] leftPath = { closeShoot, left };
  Pose2d[] forwardPath2 = { left, note2 };
  Pose2d[] backToSpeaker2 = { note2, closeShoot };

  /** Creates a new ShootForward. */
  public RedCenterThree(DriveBase driveBase) {
    // Add your commands in the addCommands() call
    addCommands(
      CustomNamedCommands.getCommand("Shoot"),
      Commands.parallel(
        CustomNamedCommands.getCommand("IntakeIn"),
        new FollowBezier(driveBase, forwardPath1, 55, true).withTimeout(3)
      ),
      Commands.parallel(
        CustomNamedCommands.getCommand("PassOff"),
        new FollowBezier(driveBase, backToSpeaker1, 55, false).withTimeout(3)
      ),
      CustomNamedCommands.getCommand("SmartShoot"), // shoot 2nd note then go get 3rd
      new FollowBezier(driveBase, leftPath, 55, false).withTimeout(3),
      Commands.parallel(
        CustomNamedCommands.getCommand("IntakeIn"),
        new FollowBezier(driveBase, forwardPath2, 55, false).withTimeout(3)
      ),
      Commands.parallel(
        CustomNamedCommands.getCommand("PassOff"),
        new FollowBezier(driveBase, backToSpeaker2, 55, false).withTimeout(3)
      ),
      CustomNamedCommands.getCommand("SmartShoot")
    );
  }
}
