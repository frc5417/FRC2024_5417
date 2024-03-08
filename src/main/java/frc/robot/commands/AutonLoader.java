package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Shooter;

public class AutonLoader {
    private final DriveBase m_driveBase;
    private final Shooter m_shooter;

    private final HolonomicPathFollowerConfig holonomic_config = new HolonomicPathFollowerConfig(
        new PIDConstants(1, 0.01, 0.01), new PIDConstants(0.5, 0.01, 0.0),
        Constants.Swerve.maxModuleSpeed, Constants.DriveTrainConstants.driveBaseRadius,
        new ReplanningConfig()
    );

    private final SendableChooser<Command> autoChooser;

    public AutonLoader(DriveBase driveBase, Shooter shooter) {
        m_driveBase = driveBase;
        m_shooter = shooter;

        AutoBuilder.configureHolonomic(
            m_driveBase::getCurrentPose, m_driveBase::resetOdometry,
            m_driveBase::getRobotRelativeChassisSpeeds, m_driveBase::setAutoSpeed,
            holonomic_config, m_driveBase::shouldFlipPath, m_driveBase);

        autoChooser = AutoBuilder.buildAutoChooser();

        autoChooser.addOption("Just Score",
            Commands.race(
                Commands.race(
                  new AutoAlign(m_driveBase, m_shooter),
                  new WaitCommand(0.5)
                  ).andThen(
                    Commands.race(
                      new RunIntestine(m_shooter, -0.2),
                      new WaitCommand(.1)
                    ).andThen(
                      Commands.parallel(
                        new RunShooter(m_shooter, 1),
                        new WaitCommand(0.35).andThen(
                          new WaitCommand(0.25).andThen(
                            new RunIntestine(m_shooter, 1)
                          )
                        )
                      )
                    )
                  ), new WaitCommand(5.0)
            ));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAuton() {
        return autoChooser.getSelected();
        
        // m_driveBase.resetOdometry(new Pose2d(7.0, 4.0, new Rotation2d(0.0)));
        // return new PathPlannerAuto("test");
    }    
}