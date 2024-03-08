// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.LimelightConstants;

public class Vision extends SubsystemBase {
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private static final NetworkTableEntry tidEntry = table.getEntry("tid");
  private static final NetworkTableEntry txEntry = table.getEntry("tx");
  private static final NetworkTableEntry targetPoseEntry = table.getEntry("targetpose_cameraspace");
  private static final NetworkTableEntry priorityid = table.getEntry("priorityid");

  private boolean setID = false;

  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (DriverStation.getAlliance().isPresent()) {
      if (!setID) {
        setID = true;
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
          priorityid.setNumber(8);
        } else if ((DriverStation.getAlliance().get() == Alliance.Red)) {
          priorityid.setNumber(4);
        }
      }
    } else {
      priorityid.setValue(null);
    }
    
    if(Robot.INSTANCE.isAutonomous() || Robot.INSTANCE.isDisabled()) {
      double botpose[] = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
      if(botpose != null && botpose.length > 5 && botpose[0] != 0.0){
        RobotContainer.driveBase.makeOdom(botpose[0], botpose[1], botpose[5]);
        SmartDashboard.putString("Limelight Status:", "I see april tag no cap");
      } else {
        SmartDashboard.putString("Limelight Status:", "I dont see apriltag not finna lie");
      }
    }
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

    if (Math.sqrt(Math.pow(pose.getX() - LimelightConstants.limelightToShooterX, 2)+Math.pow(pose.getZ(), 2)) > 2.2) return Math.toDegrees(
      Math.atan(
        ((-pose.getY() + LimelightConstants.aprilTagToTarget) - LimelightConstants.limelightToShooterY)
        /
        Math.sqrt(Math.pow((pose.getX() - LimelightConstants.limelightToShooterX)/1.67, 2) + Math.pow(pose.getZ()/1.67 - LimelightConstants.limelightToShooterZ, 2))
      )
    );

    return Math.toDegrees(
      Math.atan(
        ((-pose.getY() + LimelightConstants.aprilTagToTarget) - LimelightConstants.limelightToShooterY)
        /
        Math.sqrt(Math.pow((pose.getX() - LimelightConstants.limelightToShooterX)/1.17, 2) + Math.pow(pose.getZ()/1.17 - LimelightConstants.limelightToShooterZ, 2))
      )
    );
  }

  public static double getAdjustedHorizontalAngle(){
    Pose3d pose = getTargetPose();
    double distanceToTarget = Math.sqrt(pose.getX() * pose.getX() + pose.getZ() * pose.getZ());
    return Math.toRadians(getTX()) + Math.asin(LimelightConstants.limelightToShooterX / distanceToTarget);
  }
}
