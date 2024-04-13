package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Autos.*;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.A_Star.A_Star;

public class AutonLoader {
    private final DriveBase m_driveBase;
    private final Shooter m_shooter;
    private final Intake m_intake;

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutonLoader(DriveBase driveBase, Shooter shooter, Intake intake) {
        m_driveBase = driveBase;
        m_shooter = shooter;
        m_intake = intake;

        A_Star.rectangularObstacle(Constants.Auton.BlueObstacle_TopLeft, Constants.Auton.BlueObstacle_BottomRight);
        A_Star.rectangularObstacle(Constants.Auton.RedObstacle_TopLeft, Constants.Auton.RedObstacle_BottomRight);

        RobotContainer.registerNamedCommands();

        autoChooser.addOption("None", Commands.none());
        autoChooser.addOption("BlueCenterThree", new BlueCenterThree(m_driveBase));
        autoChooser.addOption("BlueCenterTwo", new BlueCenterTwo(m_driveBase));
        autoChooser.addOption("BlueLeftTwo", new BlueLeftTwo(m_driveBase));
        autoChooser.addOption("BlueRightTwo", new BlueRightTwo(m_driveBase));
        autoChooser.addOption("RedCenterThree", new RedCenterThree(m_driveBase));
        autoChooser.addOption("RedCenterTwo", new RedCenterTwo(m_driveBase));
        autoChooser.addOption("RedLeftTwo", new RedLeftTwo(m_driveBase));
        autoChooser.addOption("RedRightTwo", new RedRightTwo(m_driveBase));
        autoChooser.addOption("RedLeftDisrupter", new RedLeftDisrupter(m_driveBase, m_intake));

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.updateValues();
    }

    public Command getAuton() {
        return autoChooser.getSelected();
    }    
}