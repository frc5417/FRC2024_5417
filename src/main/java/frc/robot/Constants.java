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

    //Wrist overcurrent safety
    public static final int maxWristPowerCycles = 500; //Each second is 10 cycles

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
    public static final int shooter1MotorID = 33;
    public static final boolean shooter1Inversion = true;
    public static final int shooter2MotorID = 34;
    public static final boolean shooter2Inversion = false;
    public static final int intestineMotorID = 32;
    public static final boolean intestineInversion = false;
    public static final int pivotMotorID = 31;
    public static final boolean pivotInversion = false;

    // 0 indexing
    public static final Integer[] driveMotorIDS = {10, 12, 16, 14}; 
    public static final Integer[] angleMotorIDS = {11, 13, 17, 15};
    public static final Integer[] CANCoderID = {3, 4, 1, 2};
    public static final Double[] motorDegrees =
     {243.4572, 26.1036, 292.23648, 283.09572};
    public static final Double[] angleOffsets = {0.0, 0.0, 0.0, 0.0};
    public static final Double[][] angleMotorPID = {
      {0.01, 0.0, 0.005}, {0.01, 0.0, 0.005}, {0.01, 0.0, 0.005}, {0.01, 0.0, 0.005}};
    public static final Double degTolerance = 0.75;
  }

  public static class ManipulatorConstants {
    // Motor SetPoints
    public static final double intakeWristSetPointMaxDelta = 0.3;
    public static final double intakeWristShootingPoint = 2.2857;
    public static final double intakeWristPassOffPoint = 1.80574; // TODO: uh oh
    public static final double intakeWristMin = 0.60;
    public static final double intakeWristMax = 27.85;
    public static final double intakeVertical = 10.0;
    
    public static final double shooterWristSetPointMaxDelta = 0.15;
    public static final double shooterWristPassOffPoint = -1.958572;
    public static final double shooterWristTrapPoint = -6.4762;
    public static final double shooterWristMin = -24.5;
    public static final double shooterWristMax = -0.4;

    // Motor PIDs
    public static final double[] intakeWristPID = { .6, 0.007, 0.015 };
    public static final double intakeWristTolerance = 0.005;
    public static final double[] shooterWristPID = {  0.5, 0.00125, 0.00125  };
    //public static final double[] shooterWristPIDUp = { 0.5, 0.00125, 0.035 };
    public static final double shooterWristTolerance = 0.004;
  
    // Motor Powers
    public static final double intakeWristMaxPower = 0.3;
    public static final double shooterWristMaxPower = 0.4;
    public static final double elevatorMaxPower = 0.9;
    public static final double intakePower = 1.0;
    public static final double intestinePower = 0.5;

    public static final int intakeLimitSwithPort = 0;
    public static final int shooterLimitSwithPort = 0;
  }
  
  public static class Swerve {
    public static final Double angularPercentage = -1.0;
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
    public static final boolean shouldFlipAuto = false;

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

    public static final double kWheelCircumference = Units.inchesToMeters(4) * Math.PI;
    public static final double kDistanceConversionFactor = kWheelCircumference * (1 / 6.12);
    public static final double kVelocityConversionFactor = kWheelCircumference * (1/60.0) * (1 / 6.12); // rpm to m/s??
  }

  public static class DriveTrainConstants {
    public static final double driveTrainWidth = 0.6604; // in meters
    public static final double driveBaseRadius = 0.3502406; // in meters

    public static final PIDConstants ROTATION_PID = new PIDConstants(0.25, 0.01, 0);
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0, 0, 0);
    public static final Integer wheels = 4;
    public static final boolean[] invertedMotors = {true, true, true, false};
    //TODO: tune pid constants
  }

  public static class Auton {
    public static final String[] paths = {"rotateInPlace, moveForward, PathPlannerTest"};
  }

  public static class LimelightConstants {
    public static final double limelightAngle = 20.0; // in degrees
    public static final double limelightToShooterZ = 0.0508; // in meters
    public static final double limelightToShooterY = 0.244475; // in meters
    public static final double limelightToShooterX = -0.38; // in meters, TODO: change
    public static final double aprilTagToTarget = 0.66675; // in meters (changed from -0.30595 to 2.086)
    public static final double aprilTagToLowerTarget = 0.62865; // in meters, TODO: change
    public static final double startingShooterDegrees = 5;
    public static final double shooterDegreeRatio = 360 / 45; // 45 : 1
    public static final double aprilTagToTargetZ = 0.01; // in meters TODO: change
  }

  public static class FieldConstants {
    public static final double length = Units.feetToMeters(54);
    public static final double width = Units.feetToMeters(27);
  } 
}
