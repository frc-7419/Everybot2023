package frc.robot.commands; 
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class Turn90 extends CommandBase {
    
    SwerveDriveFieldCentric swerveDriveFieldCentric;
    DriveBaseSubsystem driveBaseSubsystem;
    double startingAngle;
    
    public Turn90(SwerveDriveFieldCentric swerveDriveFieldCentric, DriveBaseSubsystem driveBaseSubsystem) {
        this.swerveDriveFieldCentric = swerveDriveFieldCentric;
        this.driveBaseSubsystem = driveBaseSubsystem;
        addRequirements(driveBaseSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startingAngle = driveBaseSubsystem.getYaw()+180;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveDriveFieldCentric.setModuleStatesFromChassisSpeed(new ChassisSpeeds(0, 0, SwerveConstants.maxRotationalSpeed/4));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveDriveFieldCentric.setModuleStatesFromChassisSpeed(new ChassisSpeeds(0, 0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(startingAngle - (180+driveBaseSubsystem.getYaw())) >= 90;
    }
}
