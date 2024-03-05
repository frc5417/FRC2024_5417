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
  private static final NetworkTableEntry tidEntry = table.getEntry("tid");
  private static final NetworkTableEntry txEntry = table.getEntry("tx");
  private static final NetworkTableEntry targetPoseEntry = table.getEntry("targetpose_cameraspace");
  private static final NetworkTableEntry priorityid = table.getEntry("priorityid");

  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // priorityid.set priorityid
  }

  public static Pose3d getTargetPose() {
    double[] pose = targetPoseEntry.getDoubleArray(new double[6]);
    return new Pose3d(pose[0], pose[1], pose[2],
      new Rotation3d(Math.toRadians(pose[3]), Math.toRadians(pose[4]), Math.toRadians(pose[5])));
  }

  public static double getTID() {
    return tidEntry.getDouble(-1);
  }

  public static double getTX() {
    return txEntry.getDouble(0.0);
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

    if (Math.sqrt(Math.pow(pose.getX(), 2)+Math.pow(pose.getZ(), 2)) > 2.2) return Math.toDegrees(
      Math.atan(
        ((-pose.getY() + LimelightConstants.aprilTagToTarget) - LimelightConstants.limelightToShooterY)
        /
        Math.sqrt(Math.pow(pose.getX()/1.67, 2) + Math.pow(pose.getZ()/1.67 - LimelightConstants.limelightToShooterX, 2))
      )
    );

    return Math.toDegrees(
      Math.atan(
        ((-pose.getY() + LimelightConstants.aprilTagToTarget) - LimelightConstants.limelightToShooterY)
        /
        Math.sqrt(Math.pow(pose.getX()/1.17, 2) + Math.pow(pose.getZ()/1.17 - LimelightConstants.limelightToShooterX, 2))
      )
    );
  }
}
