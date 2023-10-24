package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;

public class MoveForward extends CommandBase {
    private DriveBaseSubsystem driveBaseSubsystem;
    private SwerveDriveFieldCentric fieldCentric;
    
    public MoveForward(DriveBaseSubsystem driveBaseSubsystem, SwerveDriveFieldCentric fieldCentric) {
        this.fieldCentric = fieldCentric;
        this.driveBaseSubsystem = driveBaseSubsystem;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        // fix these values after testing
        double vx = -(10*SwerveConstants.maxTranslationalSpeedX);
        double vy = -(10*SwerveConstants.maxTranslationalSpeed);
        double rx = 0;
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rx, driveBaseSubsystem.getRotation2d());
        fieldCentric.setModuleStatesFromChassisSpeed(speeds);
    }
    @Override
    public void end(boolean interrupted) {} 

    @Override
    public boolean isFinished() {
        return false;
    }
}
