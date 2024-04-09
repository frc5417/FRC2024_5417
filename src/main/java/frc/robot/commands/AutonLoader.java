package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Autos.ShootForward;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.A_Star.A_Star;

public class AutonLoader {
    private final DriveBase m_driveBase;
    private final Shooter m_shooter;

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutonLoader(DriveBase driveBase, Shooter shooter) {
        m_driveBase = driveBase;
        m_shooter = shooter;

        A_Star.rectangularObstacle(Constants.Auton.BlueObstacle_TopLeft, Constants.Auton.BlueObstacle_BottomRight);
        A_Star.rectangularObstacle(Constants.Auton.RedObstacle_TopLeft, Constants.Auton.RedObstacle_BottomRight);

        RobotContainer.registerNamedCommands();

        autoChooser.addOption("ShootThenForward", new ShootForward(m_driveBase));

        SmartDashboard.putData(autoChooser);
        SmartDashboard.updateValues();
    }

    public Command getAuton() {
        return autoChooser.getSelected();
    }    
}