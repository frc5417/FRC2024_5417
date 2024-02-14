// Copyright (c) FIRST and other WPILib contributors. test
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverPort = 0;
    public static final int kManipulatorPort = 1;
    public static final boolean fieldCentric = true; //FRONT IS THE SIDE OPPOSITE TO BATTERY
    public static final double joystickDeadband = 0.15; //HAS TO BE TUNED A BIT
  }

  public static class MotorConstants {

    // Intake Motors
    public static final int wristMotorID = 22;
    public static final boolean wristMotorInversion = false;
    public static final int intakeMotorID = 24;
    public static final boolean intakeMotorInversion = false;

    // Elevator Electronics
    public static final Integer elevatorMotorID = 50;
    public static final Boolean elevatorMotorInversion = false;
    public static final int throughBoreEncPort = 3;

    // Shooter Motors
    public static final int shooter1MotorID = 61;
    public static final boolean shooter1Inversion = false;
    public static final int shooter2MotorID = 61;
    public static final boolean shooter2Inversion = false;
    public static final int intestineMotorID = 61;
    public static final boolean intestineInversion = false;
    public static final int pivotMotorID = 61;
    public static final boolean pivotInversion = false;

    // 0 indexing
    public static final Integer[] driveMotorIDS = {10, 12, 14, 16}; 
    public static final Integer[] angleMotorIDS = {11, 13, 15, 17};
    public static final Integer[] CANCoderID = {3, 4, 2, 1};
    public static final Double[] motorDegrees = {130.0, 40.0, 200.0, 40.0};
    public static final Double[] angleOffsets = {0.0, 0.0, 0.0, 0.0};
    public static final Double[][] angleMotorPID = {
      {0.01, 0.0, 0.005}, {0.01, 0.0, 0.005}, {0.01, 0.0, 0.005}, {0.01, 0.0, 0.005}};
    public static final Double degTolerance = 0.75;
  }

  public static class ManipulatorConstants {
    public static final double wristTolerance = 0.05;
    public static final double wristMax = 500.0;
    public static final double wristMin = -500.0;
    
    // TUNE ALL OF THESE VALUES BASED ON PID, ONE PLAYER SUBSTATION ONLY
    public static final double wristICubeI = 0.0;
    public static final double wristConeICubeO = 0.14;
    public static final double wristConeOCubeI = 0.25;
    
    // Max Powers
    public static final double wristMaxPower = 0.9;
    public static final double elevatorMaxPower = 0.5;
    public static final double intakePower = 0.9;
    public static final double intestinePower = 0.5;

    public static final int intakeLimitSwithPort = 0;

    public static final double shooterPivotP = 0.2;
    public static final double shooterPivotI = 0.2;
    public static final double shooterPivotD = 0.2;
  }
  
  public static class Swerve {
    public static final Double angularPercentage = 1.0;
    public static final Double XPercentage = -1.0;
    public static final Double YPercentage = - 1.0;

    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;
    public static final double maxVelocity = 0.5; // m/s
    public static final double maxAcceleration = (Double) 0.025; //m/s^2
    public static final double maxAngularVelocity = 1; //rad/sec
    public static final double maxAngularAcceleration = (Double) 0.1; //rad/sec^2
    public static final double maxModuleSpeed = (Double) 0.5;
    public static final double driveBaseRadius = (Double) 0.51 * Math.sqrt(2); 
    public static final boolean shouldFlipAuto = true;

    //velocity PID tuning for overall swerve
    public static final double velocitykP = 1.0; // 0.0001
    public static final double velocitykI = 0.0;
    public static final double velocitykD = 0.0;
    public static final double aVelocitykP = 1.0;
    public static final double aVelocitykI = 0.0;
    public static final double aVelocitykD = 0.0;

    // public static final double odomProportionality = -0.93409848871;

    // public static final PathConstraints AUTON_CONSTRAINTS = new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration); // max velocity and acceleration during auton
    public static final long CommandDuration = 1000;
  }

  public static class DriveTrainConstants {
    public static final double DRIVETRAIN_WIDTH = 0.6604; // in meters
  
    public static final PIDConstants ROTATION_PID = new PIDConstants(0, 0, 0);
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0, 0, 0);
    public static final Integer wheels = 4;
    public static final boolean[] invertedMotors = {true, true, false, true};
    //TODO: tune pid constants
  }

  public static class Auton {
    public static final String[] paths = {"rotateInPlace, moveForward, PathPlannerTest"};
  }

  // public static class VisionConstants {
  //   public static final Transform3d robotToCam =
  //           new Transform3d(
  //                  new Translation3d(0.5, 0.0, 0.5),
  //                   new Rotation3d(
  //                           0, 0,
  //                           0)); 

  //   // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  //   public static final String cameraName = "OV5647";
  //   public static final double maxDistanceAway = 2.0;
  //   public static final double forwardKP = 0.1;
  //   public static final double forwardToAngleRatio = 0.5;
    
  //   public static final double CAMERA_HEIGHT_METERS = 0.72;
  //   public static final double TARGET_HEIGHT_METERS = 0;
  //   public static final double CAMERA_PITCH_RADIANS = 0;
  // }

  public static class FieldConstants {
    public static final double length = Units.feetToMeters(54);
    public static final double width = Units.feetToMeters(27);
  } 
}
