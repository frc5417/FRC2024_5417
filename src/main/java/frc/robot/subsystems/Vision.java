// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class Vision extends SubsystemBase {
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private static final NetworkTableEntry targetPose = table.getEntry("targetpose_cameraspace");
  // private static final NetworkTableEntry priorityid = table.getEntry("priorityid");

  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Pose3d getTargetPose() {
    double[] pose = targetPose.getDoubleArray(new double[6]);
    return new Pose3d(pose[0], pose[1], pose[2], new Rotation3d(pose[3], pose[4], pose[5]));
  }

  public static double getTargetX() {
    return getTargetPose().getX();
  }

  public static double getTargetY() {
    return getTargetPose().getY();
  }
  
  public static double getTargetZ() {
    return getTargetPose().getZ();
  }

  public static double getTargetShooterAngle() {
    Pose3d pose = getTargetPose();

    return Math.toDegrees(
      Math.atan(
        (pose.getY() - LimelightConstants.limelightToShooterY)
        /
        Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getZ() - LimelightConstants.limelightToShooterX, 2))
      )
    );
  }
}
