package frc.robot.subsystems.drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveModule;



public class SwerveDriveOdometry extends CommandBase {
    private GyroSubsystem gyroSubsystem;
    private SwerveModule swerveModule;
    private DriveBaseSubsystem drivebaseSubsystem;
    private SwerveDriveKinematics swerveDriveKinematics;
    private SwerveDriveOdometry m_odometry;
    private Pose2d m_pose;
    
    public SwerveDriveOdometry(SwerveModule swerveModule, SwerveDriveKinematics swerveDriveKinematics, DriveBaseSubsystem driveBaseSubsystem
                                , GyroSubsystem gyroSubsystem) {
        this.swerveModule = swerveModule;
        this.gyroSubsystem = gyroSubsystem;
        this.swerveDriveKinematics = swerveDriveKinematics;
        this.drivebaseSubsystem = driveBaseSubsystem;
        addRequirements(drivebaseSubsystem);
    }
    


    
    @Override
    public void initialize(){
       m_odometry = new SwerveDriveOdometry(swerveModule, swerveDriveKinematics, drivebaseSubsystem, gyroSubsystem);
    }

    @Override
    public void execute() {
        m_pose = m_odometry.update(swerveModule.getRotation2d(),
        new SwerveModulePosition[] {
            drivebaseSubsystem.getSwerveModule(0).getSwerveModulePosition(),
            drivebaseSubsystem.getSwerveModule(1).getSwerveModulePosition(),
            drivebaseSubsystem.getSwerveModule(2).getSwerveModulePosition(),
            drivebaseSubsystem.getSwerveModule(3).getSwerveModulePosition()
        });
    }

    



    private Pose2d update(Rotation2d rotation2d, SwerveModulePosition[] swerveModulePositions) {
        return null;
    }




  





    }

