package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;

public class AutoDock extends CommandBase {
    private DriveBaseSubsystem driveBaseSubsystem;
    private SwerveDriveFieldCentric fieldCentric;
    private double startingPitch;
    private boolean starting = true;
    
    public AutoDock(DriveBaseSubsystem driveBaseSubsystem, SwerveDriveFieldCentric fieldCentric) {
        this.fieldCentric = fieldCentric;
        this.driveBaseSubsystem = driveBaseSubsystem;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void initialize() {
        startingPitch = driveBaseSubsystem.getPitch();
    }

    @Override
    public void execute(){
        int flip = 1;
        double tiltError = driveBaseSubsystem.getPitch() - startingPitch;
        double output;
        SmartDashboard.putNumber("pitch", driveBaseSubsystem.getPitch());
        SmartDashboard.putBoolean("starting", starting);
        starting = Math.abs(driveBaseSubsystem.getPitch()-startingPitch)>10?false:starting;
        if(starting) {
            output = 0.6;
        }
        else {
            if(tiltError>1){
                output = -Math.abs(tiltError)*0.015*flip;
              }
              else if(tiltError<-1){
                output = Math.abs(tiltError)*0.015*flip;
              }
              else {
                output = 0;
              }
            output = MathUtil.clamp(output, -0.3,0.3);
        }
        fieldCentric.setModuleStatesFromChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(output*Constants.SwerveConstants.maxTranslationalSpeed,0,0,driveBaseSubsystem.getRotation2d()));

    }
    @Override
    public void end(boolean interrupted) {
        fieldCentric.setModuleStatesFromChassisSpeed(new ChassisSpeeds(0,0,0));
        driveBaseSubsystem.brake();
    } 

    @Override
    public boolean isFinished() {
        return false;
    }
}
