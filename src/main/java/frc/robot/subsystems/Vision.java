// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Regression;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.LimelightConstants;

public class Vision extends SubsystemBase {
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private static final NetworkTableEntry tidEntry = table.getEntry("tid");
  private static final NetworkTableEntry txEntry = table.getEntry("tx");
  private static final NetworkTableEntry tyEntry = table.getEntry("ty");
  private static final NetworkTableEntry targetPoseEntry = table.getEntry("targetpose_cameraspace");
  private static final NetworkTableEntry priorityid = table.getEntry("priorityid");

  private boolean setID = false;

  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (getTID() > 0) {
      Pose3d pose = getTargetPose();
      double distance = Math.sqrt(Math.pow((pose.getX() - Units.inchesToMeters(6)), 2)+Math.pow((pose.getZ() + LimelightConstants.limelightToShooterZ), 2));
      SmartDashboard.putNumber("DistanceToTarget", distance);
    } else {
      SmartDashboard.putNumber("DistanceToTarget", -1.0d);
    }

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
      priorityid.setNumber(0);
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

  public static double getTY() {
    return tyEntry.getDouble(0.0);
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

  public static double[] coeffs = Regression.quadRegression(LimelightConstants.distanceDataX, LimelightConstants.shooterAngleY);

  public static double getTargetShooterPosition() {
    Pose3d pose = getTargetPose();
    double x = Math.sqrt(Math.pow((pose.getX() - Units.inchesToMeters(6)), 2)+Math.pow((pose.getZ() + LimelightConstants.limelightToShooterZ), 2));
    double result = new PolynomialFunction(coeffs).value(x);
    return result;
  }

  //adjust 0.75
  public static double getAdjustedHorizontalAngle(){
    Pose3d pose = getTargetPose();
    double distanceToTarget = Math.sqrt(pose.getX() * pose.getX() + pose.getZ() * pose.getZ());
    return Math.toRadians(getTX()) + Math.asin(LimelightConstants.limelightToShooterX / distanceToTarget);
  }
}
