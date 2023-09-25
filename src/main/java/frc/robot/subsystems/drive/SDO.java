package frc.robot.subsystems.drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class SDO extends CommandBase {
    private GyroSubsystem gyroSubsystem;
    private DriveBaseSubsystem drivebaseSubsystem;
    private SwerveDriveKinematics swerveDriveKinematics;
    private SwerveDriveOdometry m_odometry;
    private SwerveModulePosition[] positions = new SwerveModulePosition[4];
    private Pose2d m_pose;
  
    
    public SDO(SwerveDriveKinematics swerveDriveKinematics, DriveBaseSubsystem driveBaseSubsystem
                                , GyroSubsystem gyroSubsystem) {
        this.gyroSubsystem = gyroSubsystem;
        this.swerveDriveKinematics = swerveDriveKinematics;
        this.drivebaseSubsystem = driveBaseSubsystem;
        
        positions[0] = drivebaseSubsystem.getSwerveModule(0).getPosition();
        positions[1] = drivebaseSubsystem.getSwerveModule(1).getPosition();
        positions[2] = drivebaseSubsystem.getSwerveModule(2).getPosition();
        positions[3] = drivebaseSubsystem.getSwerveModule(3).getPosition();

        addRequirements(drivebaseSubsystem);
    }
    
    
    @Override
    public void initialize(){
       m_odometry = new SwerveDriveOdometry(swerveDriveKinematics, new Rotation2d(gyroSubsystem.getAngle()), positions, new Pose2d(0, 0, new Rotation2d(0)));
    }

    @Override
    public void execute() {
        positions[0] = drivebaseSubsystem.getSwerveModule(0).getPosition();
        positions[1] = drivebaseSubsystem.getSwerveModule(1).getPosition();
        positions[2] = drivebaseSubsystem.getSwerveModule(2).getPosition();
        positions[3] = drivebaseSubsystem.getSwerveModule(3).getPosition();
        m_pose = m_odometry.update(new Rotation2d(gyroSubsystem.getAngle()), positions);
    }

    }

