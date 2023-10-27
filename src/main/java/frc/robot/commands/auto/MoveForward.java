package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;

public class MoveForward extends CommandBase {
    private DriveBaseSubsystem driveBaseSubsystem;
    private SwerveDriveFieldCentric fieldCentric;
    private double speed;
    private double meters;
    
    public MoveForward(DriveBaseSubsystem driveBaseSubsystem, SwerveDriveFieldCentric fieldCentric, double speed, double meters) {
        this.fieldCentric = fieldCentric;
        this.speed = speed;
        this.meters = meters;
        this.driveBaseSubsystem = driveBaseSubsystem;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void initialize() {
        driveBaseSubsystem.resetDriveEnc();
    }

    @Override
    public void execute(){
        fieldCentric.setModuleStatesFromChassisSpeed(new ChassisSpeeds(speed, 0, 0));
    }
    @Override
    public void end(boolean interrupted) {
        fieldCentric.setModuleStatesFromChassisSpeed(new ChassisSpeeds(0, 0, 0));
    } 

    @Override
    public boolean isFinished() {
        return driveBaseSubsystem.reachedDist(meters);
    }
}
