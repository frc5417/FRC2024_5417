// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedCenterThree extends SequentialCommandGroup {
  Pose2d startPose = new Pose2d(15.33, 5.55, Rotation2d.fromDegrees(0)); //right in front of speaker
  Pose2d endPose = new Pose2d(13.24, 5.55, Rotation2d.fromDegrees(0)); //right in front of speaker

  Pose2d[] forwardPath1 =  { startPose, endPose };

  /** Creates a new ShootForward. */
  public RedCenterThree(DriveBase driveBase) {
    // Add your commands in the addCommands() call
    addCommands(
      new FollowBezier(driveBase, forwardPath1, 70, true)
    );
  }
}
