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
public class ShootForward extends SequentialCommandGroup {
  Pose2d startPose = new Pose2d(1.1, 5.4, Rotation2d.fromDegrees(0));
  Pose2d endPose = new Pose2d(2.88, 5.4, Rotation2d.fromDegrees(0));

  Pose2d[] path =  {startPose, endPose };

  /** Creates a new ShootForward. */
  public ShootForward(DriveBase driveBase) {
    // Add your commands in the addCommands() call
    addCommands(
      CustomNamedCommands.getCommand("Shoot"),
      new FollowBezier(driveBase, path, 500, true)
    );
  }
}
