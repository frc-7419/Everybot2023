package frc.robot.commands.auto; 
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class Turn180 extends CommandBase {
    
    SwerveDriveFieldCentric swerveDriveFieldCentric;
    DriveBaseSubsystem driveBaseSubsystem;
    
    public Turn180(SwerveDriveFieldCentric swerveDriveFieldCentric, DriveBaseSubsystem driveBaseSubsystem) {
        this.swerveDriveFieldCentric = swerveDriveFieldCentric;
        this.driveBaseSubsystem = driveBaseSubsystem;
        addRequirements(driveBaseSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // turn the robot 180 degrees
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 180*SwerveConstants.maxRotationalSpeed, driveBaseSubsystem.getRotation2d());
        swerveDriveFieldCentric.setModuleStatesFromChassisSpeed(speeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
