package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it u.nder the terms of
// the WPILib BSD license file in the root directory of this project.

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.combined.Intestine;
import frc.robot.commands.combined.PassOffPoint;
import frc.robot.subsystems.*;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static AHRS ahrs = new AHRS(SerialPort.Port.kMXP);
  public static Kinematics kinematics = new Kinematics(ahrs);

  // Robot Subsystems
  public static DriveBase driveBase = new DriveBase(kinematics, ahrs);
  public static Intake intake = new Intake();
  public static Shooter shooter = new Shooter();
  public static Elevator elevator = new Elevator();
  public static Vision vision = new Vision();

  // Robot Commands
  public static AutonLoader autonLoader;
  public static TeleopDrive teleopDrive = new TeleopDrive(driveBase/* , manipulator, elevator */); // ALL SUBSYSTEMS
  public static ToggleIntake intakeOut = new ToggleIntake(intake, 1);
  public static ToggleIntake intakeIn = new ToggleIntake(intake, -1);
  public static IntakeWristSetPoint intakeShootingPoint = new IntakeWristSetPoint(intake,
      Constants.ManipulatorConstants.intakeWristShootingPoint);
  public static IntakeWristJoystick intakeWristtJoystick = new IntakeWristJoystick(intake);
  public static ShooterWristJoystick shooterWristJoystick = new ShooterWristJoystick(shooter);
  public static PassOffPoint passOffPoint = new PassOffPoint(intake, shooter);
  public static Intestine intestineForward = new Intestine(shooter);
  public static RunIntestine intestineBackward = new RunIntestine(shooter, -1);
  public static ElevatorJoystick elevatorJoystick = new ElevatorJoystick(elevator);
  public static AutoAlign autoAlign = new AutoAlign(driveBase, shooter);
  public static ShooterWristSetPoint shooterTrap = new ShooterWristSetPoint(shooter,
      Constants.ManipulatorConstants.shooterWristTrapPoint);
  public static Command shoot = Commands.sequence(
      Commands.race(
          new IntakeWristSetPoint(intake, Constants.ManipulatorConstants.intakeVertical, true),
          new RunIntestine(shooter, -0.15),
          new WaitCommand(0.15)),
      Commands.parallel(
          Commands.race(
            autoAlign,
            new WaitCommand(0.5)
          ),
          Commands.race(
              new RunIntestine(shooter, -0.1),
              new WaitCommand(0.15)
              ).andThen(
                  Commands.parallel(
                      new RunShooter(shooter, 1),
                      Commands.race(
                          new WaitCommand(0.3).andThen(
                              new RunIntestine(shooter, 1)))))));

  public static Command shootManual = Commands.race(
      new RunIntestine(shooter, -0.2),
      new WaitCommand(.1)).andThen(
          Commands.parallel(
              new RunShooter(shooter, 1),
              new WaitCommand(0.35).andThen(
                  new WaitCommand(0.25).andThen(
                      new RunIntestine(shooter, 1)))));

  public static Command alignAndShoot = Commands.sequence(
      Commands.race(
          new IntakeWristSetPoint(intake, 0, true),
          new WaitCommand(0.25)),
      Commands.race(
          Commands.parallel(
              new RunIntestine(shooter, -0.1),
              new ShooterWristSetPoint(shooter, -3.872851)),
          new WaitCommand(.3)).andThen(
              Commands.parallel(
                  Commands.race(
                      new RunIntestine(shooter, -0.1),
                      new WaitCommand(.1)),
                  new RunShooter(shooter, 1),
                  new WaitCommand(0.35).andThen(
                      new WaitCommand(0.25).andThen(
                          new RunIntestine(shooter, 1))))));
  
  private final static CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverPort);
  private final static CommandXboxController m_manipulatorController = new CommandXboxController(
      OperatorConstants.kManipulatorPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Register Named Commands
    NamedCommands.registerCommand("Shoot",
    Commands.race(
      Commands.sequence(
        Commands.race(
            new IntakeWristSetPoint(intake, 0, true),
            new WaitCommand(0.25)),
        Commands.race(
            Commands.parallel(
                new RunIntestine(shooter, -0.15),
                new ShooterWristSetPoint(shooter, -3.872851)),
            new WaitCommand(.3)).andThen(
                Commands.parallel(
                    Commands.race(
                        new RunIntestine(shooter, -0.2),
                        new WaitCommand(.15)),
                    new RunShooter(shooter, 1),
                    new WaitCommand(0.35).andThen(
                        new WaitCommand(0.25).andThen(
                            new RunIntestine(shooter, 1)))))),
      new WaitCommand(2.25)
    ));

    NamedCommands.registerCommand("SmartShoot", Commands.sequence(
      Commands.race(
          new IntakeWristSetPoint(intake, Constants.ManipulatorConstants.intakeVertical, true),
          new RunIntestine(shooter, -0.15),
          new WaitCommand(0.20)),
      Commands.parallel(
          Commands.race(
            new AutoAlign(driveBase, shooter),
            new WaitCommand(0.5)
          ),
          Commands.race(
              new RunIntestine(shooter, -0.15),
              new WaitCommand(0.25)
              ).andThen(
                  Commands.parallel(
                      new RunShooter(shooter, 1),
                      Commands.race(
                          new WaitCommand(0.35).andThen(
                              new RunIntestine(shooter, 1))))))));

    NamedCommands.registerCommand("DriveForward",
      Commands.race(
        Commands.sequence(
          Commands.race(
            new IntakeWristSetPoint(intake, 27.8, true),
            new WaitCommand(0.5)
          ),
          Commands.parallel(
            new RawDrive(driveBase, 0, 0.5, 0, 170),
            Commands.race(
              new ToggleIntake(intake, -0.665),
              new WaitCommand(3.6)
            )
          )
        ),
        new WaitCommand(7.0)
    ));

    NamedCommands.registerCommand("DriveForwardShort",
      Commands.race(
        Commands.sequence(
          Commands.race(
            new IntakeWristSetPoint(intake, 27.8, true),
            new WaitCommand(0.5)
          ),
          Commands.parallel(
            new RawDrive(driveBase, 0, 0.5, 0, 50),
            Commands.race(
              new ToggleIntake(intake, -0.65),
              new WaitCommand(2.6)
            )
          )
        ),
        new WaitCommand(7.0)
    ));

    NamedCommands.registerCommand("DriveBackwardsShort",
      Commands.race(
        Commands.sequence(
          Commands.race(
            new IntakeWristSetPoint(intake, 27.8, true),
            new WaitCommand(0.5)
          ),
          Commands.parallel(
            new RawDrive(driveBase, 0, 0.5, 0, 50),
            Commands.race(
              new ToggleIntake(intake, -1),
              new WaitCommand(2.6)
            )
          )
        ),
        new WaitCommand(7.0)
    ));

    NamedCommands.registerCommand("DriveFL",
      Commands.race(
        Commands.sequence(
          Commands.race(
            new IntakeWristSetPoint(intake, 27.8, true),
            new WaitCommand(0.5)
          ),
          Commands.parallel(
            new RawDrive(driveBase, -0.5, 0.5, 0, 60),
            Commands.race(
              new ToggleIntake(intake, -0.4),
              new WaitCommand(2.6)
            )
          )
        ),
        new WaitCommand(7.0)
    ));

    NamedCommands.registerCommand("DriveBR",
      Commands.race(
        Commands.sequence(
          Commands.race(
            new IntakeWristSetPoint(intake, 27.8, true),
            new WaitCommand(0.5)
          ),
          Commands.parallel(
            new RawDrive(driveBase, 0.5, -0.5, 0, 60),
            Commands.race(
              new ToggleIntake(intake, -0.4),
              new WaitCommand(2.6)
            )
          )
        ),
        new WaitCommand(7.0)
    ));

    NamedCommands.registerCommand("DriveFR",
      Commands.race(
        Commands.sequence(
          Commands.race(
            new IntakeWristSetPoint(intake, 27.8, true),
            new WaitCommand(0.5)
          ),
          Commands.parallel(
            new RawDrive(driveBase, 0.5, 0.5, 0, 60),
            Commands.race(
              new ToggleIntake(intake, -0.4),
              new WaitCommand(2.6)
            )
          )
        ),
        new WaitCommand(7.0)
    ));

    NamedCommands.registerCommand("DriveBL",
      Commands.race(
        Commands.sequence(
          Commands.race(
            new IntakeWristSetPoint(intake, 27.8, true),
            new WaitCommand(0.5)
          ),
          Commands.parallel(
            new RawDrive(driveBase, -0.5, -0.5, 0, 60),
            Commands.race(
              new ToggleIntake(intake, -0.4),
              new WaitCommand(2.6)
            )
          )
        ),
        new WaitCommand(7.0)
    ));

    NamedCommands.registerCommand("PassOff",
      Commands.race(
        new PassOffPoint(intake, shooter),
        new WaitCommand(3.0)
    ));

    NamedCommands.registerCommand("AutoAlignX", 
    Commands.race(
      new AutoAlignX(driveBase),
      new WaitCommand(0.8)
    ));


    NamedCommands.registerCommand("DriveBackward", 
    Commands.race(
      new RawDrive(driveBase, 0, -0.5, 0, 135),
      new WaitCommand(6.0)
    ));

    autonLoader = new AutonLoader(driveBase, shooter);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    m_driverController.b().whileTrue(new RunShooter(shooter, -.15));
    m_manipulatorController.povUp().whileTrue(intestineForward).whileTrue(intakeOut);
    m_manipulatorController.povDown().whileTrue(intestineBackward);
    m_manipulatorController.povLeft().whileTrue(shooterTrap);
    m_manipulatorController.povRight().whileTrue(shootManual);
    m_manipulatorController.y().whileTrue(shoot);
    m_manipulatorController.x().whileTrue(
        Commands.parallel(
            new RunShooter(shooter, 0.5),
            Commands.race(
                new RunIntestine(shooter, -0.2),
                new WaitCommand(.2)).andThen(
                    new RunIntestine(shooter, 1))));
    m_manipulatorController.a().whileTrue(alignAndShoot);
    m_manipulatorController.b().whileTrue(passOffPoint);
    m_manipulatorController.rightTrigger(Constants.OperatorConstants.joystickDeadband).whileTrue(intakeOut);
    m_manipulatorController.leftTrigger(Constants.OperatorConstants.joystickDeadband).whileTrue(intakeIn);
  }

  public static void setDriverRumble(double rumbleVal) {
    m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, rumbleVal);
  }

  public static double getRightJoyX() {
    if (Math.abs(m_driverController.getRightX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_driverController.getRightX() * 0.5;
    } else {
      return 0;
    }
  }

  // public static void setManipulatorRumble(double rumbleVal) {
  // m_manipulatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble,
  // rumbleVal);
  // }

  public static double getDriverLeftJoyX() {
    if (Math.abs(m_driverController.getLeftX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_driverController.getLeftX();
    } else {
      return 0;
    }
  }

  public static double getDriverLeftJoyY() {
    if (Math.abs(m_driverController.getLeftY()) > Constants.OperatorConstants.joystickDeadband) {
      return -m_driverController.getLeftY();
    } else {
      return 0;
    }
  }

  public static double getDriverRightJoyX() {
    if (Math.abs(m_driverController.getRightX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_driverController.getRightX();
    } else {
      return 0;
    }
  }

  public static double getDriverRightJoyY() {
    if (Math.abs(m_driverController.getRightY()) > Constants.OperatorConstants.joystickDeadband) {
      return m_driverController.getRightY();
    } else {
      return 0;
    }
  }

  // =========================================================
  public static double getManipulatorLeftJoyY() {
    if (Math.abs(m_manipulatorController.getLeftY()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getLeftY();
    } else {
      return 0;
    }
  }

  public static double getManipulatorLeftJoyX() {
    if (Math.abs(m_manipulatorController.getLeftX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getLeftX();
    } else {
      return 0;
    }
  }

  public static double getManipulatorRightJoyY() {
    if (Math.abs(m_manipulatorController.getRightY()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getRightY();
    } else {
      return 0;
    }
  }

  public static double getManipulatorRightJoyX() {
    if (Math.abs(m_manipulatorController.getRightX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getRightX();
    } else {
      return 0;
    }
  }

  public static double getManipulatorRightTrigger() {
    if (Math.abs(m_manipulatorController.getRightTriggerAxis()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getRightTriggerAxis();
    } else {
      return 0;
    }
  }

  public static double getManipulatorLeftTrigger() {
    if (Math.abs(m_manipulatorController.getLeftTriggerAxis()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getLeftTriggerAxis();
    } else {
      return 0;
    }
  }

  // public static Boolean getManipulatorBBool() {
  // return m_manipulatorController.b().getAsBoolean();
  // }

  // public static Boolean getManipulatorABool() {
  // return m_manipulatorController.a().getAsBoolean();
  // }

  // public static Boolean getManipulatorXBool() {
  // return m_manipulatorController.x().getAsBoolean();
  // }

  // public static Boolean getManipulatorYBool() {
  // return m_manipulatorController.y().getAsBoolean();
  // }

  public static Boolean getManipulatorLeftBumperBool() {
    return m_manipulatorController.leftBumper().getAsBoolean();
  }

  public static Boolean getManipulatorRightBumperBool() {
    return m_manipulatorController.rightBumper().getAsBoolean();
  }

  public static Boolean getDPadUp() {
    return m_manipulatorController.povUp().getAsBoolean();
  }

  public static long getFPGATime() {
    return HALUtil.getFPGATime();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonLoader.getAuton();
  }

  public void runTeleopCommand() {
    teleopDrive.schedule();
    elevatorJoystick.schedule();
    intakeWristtJoystick.schedule();
    shooterWristJoystick.schedule();

  }

  public void stopTeleopCommand() {
    teleopDrive.cancel();
    elevatorJoystick.cancel();
    intakeWristtJoystick.cancel();
    shooterWristJoystick.cancel();
  }

  // public static void setLEDsOff() {
  // m_lightsControl.setLightConfig(3);
  // }

  // public static void setLEDsOn() {
  // m_lightsControl.setLightConfig(0);
  // }

  public static double findClockTime(double seconds) {
    double clocktime = (seconds / 0.02);
    return clocktime;
  }

  public static ChassisSpeeds getSaturatedSpeeds(double xVel, double yVel, double omega) {
    return new ChassisSpeeds(xVel * Constants.Swerve.XPercentage, yVel * Constants.Swerve.YPercentage,
        omega * Constants.Swerve.angularPercentage);
  }
}
