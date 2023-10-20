package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;

public class AutoDock extends CommandBase {
    PIDController pitchController;
    private DriveBaseSubsystem driveBaseSubsystem;
    private SwerveModule swerveModule;
    private SwerveDriveFieldCentric fieldCentric;
    private double recordedPitch;

    //constants
    private double SETPOINT = 0;
    private double TOLERANCE = 0.1;
    
    public AutoDock(DriveBaseSubsystem driveBaseSubsystem, SwerveModule swerveModule, SwerveDriveFieldCentric fieldCentric) {
        this.pitchController = new PIDController(0.01, 0, 0);
        this.fieldCentric = fieldCentric;
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.swerveModule = swerveModule;
        addRequirements(driveBaseSubsystem);
    }
    public ChassisSpeeds getChassisSpeeds(double vx){
        double vy = 1;
        double rx = Math.PI/6;

        //WPILIB does the Field-Relative Conversions for you, easy peas y
        
        return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rx, driveBaseSubsystem.getRotation2d());
    }
    public ChassisSpeeds endSpeed(){
        return ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0,driveBaseSubsystem.getRotation2d());
    }

    @Override
    public void initialize() {
        pitchController.setSetpoint(SETPOINT);
        pitchController.setTolerance(TOLERANCE);
        recordedPitch = fieldCentric.recordedAngle;
    }

    @Override
    public void execute(){
        double pitch = driveBaseSubsystem.getPitch();
        SmartDashboard.putNumber("pitch", pitch);
        double output = pitchController.calculate(pitch);
            fieldCentric.setModuleStates(fieldCentric.ChassisSpeedstoModuleSpeeds(getChassisSpeeds(output)));

    }
    @Override
    public void end(boolean interrupted) {} 

    @Override
    public boolean isFinished() {
        return pitchController.atSetpoint();
    }
}
