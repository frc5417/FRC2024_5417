package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class AutonLoader {
    private final DriveBase m_driveBase;
    private final HolonomicPathFollowerConfig holonomic_config = new HolonomicPathFollowerConfig(
        new PIDConstants(0.1, 0.0, 0.0), new PIDConstants(0.1, 0.0, 0.0),
        Constants.Swerve.maxModuleSpeed, Constants.DriveTrainConstants.driveBaseRadius,
        new ReplanningConfig()
    );

    private final SendableChooser<Command> autoChooser;

    public AutonLoader(DriveBase driveBase) {

        m_driveBase = driveBase;

        AutoBuilder.configureHolonomic(
            m_driveBase::getCurrentPose, m_driveBase::resetOdometry,
            m_driveBase::getRobotRelativeChassisSpeeds, m_driveBase::setAutoSpeed,
            holonomic_config, m_driveBase::shouldFlipPath, m_driveBase);

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAuton() {
        return autoChooser.getSelected();
        
        // m_driveBase.resetOdometry(new Pose2d(7.0, 4.0, new Rotation2d(0.0)));
        // return new PathPlannerAuto("test");
    }    
}