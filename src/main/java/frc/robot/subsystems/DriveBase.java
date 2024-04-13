package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase extends SubsystemBase {

    private static Module.ModuleState targetModuleStates[];
    private final Kinematics m_kinematics;
    private final AHRS m_ahrs;

    public static Module[] moduleGroup;

    public static double[] odomDeltas = {0, 0, 0, 0};
    public static double[] odomPrevDeltas = {0, 0, 0, 0};
    public static double[] odomAngles = {0, 0, 0, 0};
    public static double[] encoderOffset = {0, 0, 0, 0};
    public static double[] encoderDriveOffset = {0, 0, 0, 0};

    double mod1Prev = 0;
    double mod1Curr = 0;
    int counter = 0;

    Translation2d m_frontLeftLocation = new Translation2d(-0.23495, 0.23495);
    Translation2d m_frontRightLocation = new Translation2d(0.23495, 0.23495);
    Translation2d m_backLeftLocation = new Translation2d(-0.23495, -0.23495);
    Translation2d m_backRightLocation = new Translation2d(0.23495, -0.23495);

    SwerveDriveKinematics m_skdKine = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    SwerveDriveOdometry m_sdkOdom;

    double X = 0.0;
    double Y = 0.0;

    ChassisSpeeds autoSetSpeed = new ChassisSpeeds();

    boolean hasUpdatedOffset = true;
    double savedOffetPos = 0.0d;

    public DriveBase(Kinematics kinematics, AHRS ahrs) {
        m_kinematics = kinematics;
        m_ahrs = ahrs;
        // m_ahrs.reset();
        setAngleOffsetTeleop(0);

        moduleGroup = new Module[4];
        for (int i = 0; i < 4; i++) {
            moduleGroup[i] = new Module(i, Constants.DriveTrainConstants.invertedMotors[i]);
            encoderOffset[i] = moduleGroup[i].getAngleInRadians();
            encoderDriveOffset[i] = moduleGroup[i].integratedDriveEncoder.getPosition();
        }

        targetModuleStates = new Module.ModuleState[4];

        for (int i = 0; i < 4; i++)
            targetModuleStates[i] = new Module.ModuleState(0, Constants.MotorConstants.motorDegrees[i] * (Math.PI/180));

        makeOdom(0, 0, 0);
    }

    public void makeOdom(double x, double y, double yaw){
        SmartDashboard.putString("Making odom", x + " " + y + " " + yaw);
        m_sdkOdom = new SwerveDriveOdometry(
            m_skdKine, m_ahrs.getRotation2d(), new SwerveModulePosition[] {
                new SwerveModulePosition(odomDeltas[3], new Rotation2d(odomAngles[3])),
                new SwerveModulePosition(odomDeltas[2], new Rotation2d(odomAngles[2])),
                new SwerveModulePosition(odomDeltas[1], new Rotation2d(odomAngles[1])),
                new SwerveModulePosition(odomDeltas[0], new Rotation2d(odomAngles[0]))
            }, new Pose2d (x, y, new Rotation2d(yaw))
        );
    }

    public void setAngleOffsetTeleop(double amount) {
        hasUpdatedOffset = false;
        savedOffetPos = amount;
    }

    public Pose2d getCurrentPose() {
        Pose2d inverted = new Pose2d(m_sdkOdom.getPoseMeters().getY() * -1.0, m_sdkOdom.getPoseMeters().getX(), m_sdkOdom.getPoseMeters().getRotation());
        return inverted;
    }

    public boolean isRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public boolean shouldFlipPath() {
        // if (DriverStation.getAlliance().equals(Alliance.Blue)) {
        //     return Constants.Swerve.shouldFlipAuto;
        // } else if ((DriverStation.getAlliance().equals(Alliance.Red))) {
        //     return !Constants.Swerve.shouldFlipAuto;
        // } else {
        //     System.out.println("IDK THE ROBOT ALLIANCE BRO"); 
        //     return false;
        // }
        
        return Constants.Swerve.shouldFlipAuto;
    }

    public void resetOdometry(Pose2d pose) {
        Pose2d inverted = new Pose2d(pose.getY(), pose.getX() * -1.0, pose.getRotation());
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
            odomDeltas[i] = (((moduleGroup[i].integratedDriveEncoder.getPosition() - encoderDriveOffset[i])/6.12) * (0.102*Math.PI));// - odomPrevDeltas[i];
            odomAngles[i] = smallestAngle(moduleGroup[i].getAngleInRadians());//smallestAngle(moduleGroup[i].getAngleInRadians()*(180.0/Math.PI)) * (Math.PI/180.0);
        }
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(odomDeltas[3], new Rotation2d(odomAngles[3])),
                new SwerveModulePosition(odomDeltas[2], new Rotation2d(odomAngles[2])),
                new SwerveModulePosition(odomDeltas[1], new Rotation2d(odomAngles[1])),
                new SwerveModulePosition(odomDeltas[0], new Rotation2d(odomAngles[0]))
        };
        m_sdkOdom.resetPosition(m_ahrs.getRotation2d(), modulePositions, inverted);
    }

    public void setHardStates(Module.ModuleState[] targetState) {
        targetModuleStates = targetState;
    }

    public void setDriveSpeed(ChassisSpeeds chassisSpeeds) {
        targetModuleStates = m_kinematics.getComputedModuleStates(chassisSpeeds);
    }

    public void setBlueAutoSpeed(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds inverted = new ChassisSpeeds(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond * -1.0, chassisSpeeds.omegaRadiansPerSecond);
        targetModuleStates = m_kinematics.getComputedModuleStates(inverted);
    }

    public void setRedAutoSpeed(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds inverted = new ChassisSpeeds(chassisSpeeds.vyMetersPerSecond * -1.0, chassisSpeeds.vxMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
        targetModuleStates = m_kinematics.getComputedModuleStates(inverted);
    }

    public void resetDrive() {
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].resetDriveAngleEncoder();
        }
    }

    public double smallestAngle(double largeAngle) {
        if (largeAngle > 0) {
            return largeAngle - Math.floor(Math.abs(largeAngle)/(2*Math.PI)) * (2*Math.PI);
        } else {
            return (largeAngle + Math.floor(Math.abs(largeAngle)/(2*Math.PI)) * (2*Math.PI)) + (2*Math.PI);
        }
    }

    @Override
    public void periodic() {
        if (!hasUpdatedOffset && Robot.INSTANCE.isTeleopEnabled()) {
            hasUpdatedOffset = true;
            m_ahrs.setAngleAdjustment(savedOffetPos);
        }

        // RobotContainer.m_photonsubsystem.updatePose();
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
            odomDeltas[i] = moduleGroup[i].integratedDriveEncoder.getPosition();
            odomAngles[i] = smallestAngle(moduleGroup[i].getAngleInRadians());
            // SmartDashboard.putNumber("Module" + i + "_Angle", moduleGroup[i].getAngle());
        }
        
        m_sdkOdom.update(m_ahrs.getRotation2d(), new SwerveModulePosition[] {
            new SwerveModulePosition(Math.abs(odomDeltas[3]), new Rotation2d(odomAngles[3])),
            new SwerveModulePosition(Math.abs(odomDeltas[2]), new Rotation2d(odomAngles[2])),
            new SwerveModulePosition(Math.abs(odomDeltas[1]), new Rotation2d(odomAngles[1])),
            new SwerveModulePosition(Math.abs(odomDeltas[0]), new Rotation2d(odomAngles[0]))
        });

        SmartDashboard.putNumber("Yaw", m_ahrs.getYaw());
        SmartDashboard.putNumber("Pitch", m_ahrs.getPitch());
        SmartDashboard.putNumber("Roll", m_ahrs.getRoll());
        SmartDashboard.putNumber("Rotations", m_ahrs.getAngle());

        SmartDashboard.putNumber("ODOM X", getCurrentPose().getX());
        SmartDashboard.putNumber("ODOM Y",getCurrentPose().getY());
        // double voltage = m_pdp.getVoltage();
        // SmartDashboard.putNumber("Voltage", voltage);

        // SmartDashboard.putNumber("Mod1_delta", Math.abs(odomDeltas[0]));
        // SmartDashboard.putNumber("Mod2_delta", Math.abs(odomDeltas[1]));
        // SmartDashboard.putNumber("Mod3_delta", Math.abs(odomDeltas[2]));
        // SmartDashboard.putNumber("Mod4_delta", Math.abs(odomDeltas[3]));

        SmartDashboard.putNumber("Mod1_theta", -Math.abs(Math.toDegrees(odomAngles[0]))-90);
        SmartDashboard.putNumber("Mod2_theta", -Math.abs(Math.toDegrees(odomAngles[1]))-90);
        SmartDashboard.putNumber("Mod3_theta", -Math.abs(Math.toDegrees(odomAngles[2]))-90);
        SmartDashboard.putNumber("Mod4_theta", -Math.abs(Math.toDegrees(odomAngles[3]))-90);
        
        SmartDashboard.updateValues();
        
    }
}
