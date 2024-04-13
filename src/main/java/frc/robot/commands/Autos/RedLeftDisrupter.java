// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CustomNamedCommands;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.IntakeWristSetPoint;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedLeftDisrupter extends SequentialCommandGroup {
  Pose2d startPose = new Pose2d(15.96, 4.47, Rotation2d.fromDegrees(240)); //right in front of speaker
  Pose2d curveCenter = new Pose2d(15.0, 2.11, Rotation2d.fromDegrees(180));
  Pose2d note1 = new Pose2d(8.6, 0.4, Rotation2d.fromDegrees(180));
  Pose2d note1Moved = new Pose2d(8.7, 0.67, Rotation2d.fromDegrees(120)); 
  Pose2d note2 = new Pose2d(8.6, 2.07, Rotation2d.fromDegrees(180));
  Pose2d note2Moved = new Pose2d(8.7, 2.34, Rotation2d.fromDegrees(120)); 
  Pose2d note3 = new Pose2d(8.6, 3.78, Rotation2d.fromDegrees(180));
  Pose2d note3Moved = new Pose2d(8.7, 4.05, Rotation2d.fromDegrees(120));
  Pose2d note4 = new Pose2d(8.6, 5.43, Rotation2d.fromDegrees(180));
  Pose2d note4Moved = new Pose2d(8.7, 5.70, Rotation2d.fromDegrees(120));  
  Pose2d note5 = new Pose2d(8.6, 7.0, Rotation2d.fromDegrees(180));
  Pose2d note5Moved = new Pose2d(8.7, 7.27, Rotation2d.fromDegrees(120));  

  Pose2d[] curveToNote1 =  { startPose, curveCenter, note1 };
  Pose2d[] moveFirstNote = { note1, note1Moved };
  Pose2d[] toSecondNote = { note1Moved, note2 };
  Pose2d[] moveSecondNote = { note2, note2Moved };
  Pose2d[] toThirdNote = { note2Moved, note3 };
  Pose2d[] moveThirdNote = { note3, note3Moved };
  Pose2d[] toFourthNote = { note3Moved, note4 };
  Pose2d[] moveFourthNote = { note4, note4Moved };
  Pose2d[] toFifthNote = { note4Moved, note5 };
  Pose2d[] moveFifthNote = { note5, note5Moved };

  public RedLeftDisrupter(DriveBase driveBase, Intake intake) {
    // Add your commands in the addCommands() call
    addCommands(
      CustomNamedCommands.getCommand("Shoot"),
      Commands.parallel(
        new IntakeWristSetPoint(intake, ManipulatorConstants.intakeWristMax),
        new FollowBezier(driveBase, curveToNote1, 250, true).withTimeout(3)
      ),
      new FollowBezier(driveBase, moveFirstNote, 55, false).withTimeout(3),
      new FollowBezier(driveBase, toSecondNote, 55, false).withTimeout(3),
      new FollowBezier(driveBase, moveSecondNote, 55, false).withTimeout(3),
      new FollowBezier(driveBase, toThirdNote, 55, false).withTimeout(3),
      new FollowBezier(driveBase, moveThirdNote, 55, false).withTimeout(3),
      new FollowBezier(driveBase, toFourthNote, 55, false).withTimeout(3),
      new FollowBezier(driveBase, moveFourthNote, 55, false).withTimeout(3),
      new FollowBezier(driveBase, toFifthNote, 55, false).withTimeout(3),
      new FollowBezier(driveBase, moveFifthNote, 55, false).withTimeout(3)
    );
  }
}
