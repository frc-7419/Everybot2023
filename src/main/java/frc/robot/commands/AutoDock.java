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
    private PIDController pitchController;
    private DriveBaseSubsystem driveBaseSubsystem;
    private SwerveDriveFieldCentric fieldCentric;
    private double recordedStartingPitch;
    private boolean toggle = true;
    private double modifier;

    //constants
    private double SETPOINT = 0;
    private double TOLERANCE = 0.5;
    
    public AutoDock(DriveBaseSubsystem driveBaseSubsystem, SwerveDriveFieldCentric fieldCentric, double modifier) {
        this.pitchController = new PIDController(0.04, 0, 0);
        this.fieldCentric = fieldCentric;
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.modifier = modifier;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void initialize() {
        recordedStartingPitch = driveBaseSubsystem.getPitch();
        pitchController.setSetpoint(SETPOINT);
        pitchController.setTolerance(TOLERANCE);
    }

    @Override
    public void execute(){
        double output;
        SmartDashboard.putNumber("pitch", driveBaseSubsystem.getPitch());
        SmartDashboard.putBoolean("toggle", toggle);
        if(Math.abs(driveBaseSubsystem.getPitch()-recordedStartingPitch)>10) toggle = false;
        if(toggle) {
            output = 1;
        }
        else {
            output = pitchController.calculate(driveBaseSubsystem.getPitch());
            output = MathUtil.clamp(output, -0.2,0.2);
        }
        fieldCentric.setModuleStatesFromChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(modifier*output*Constants.SwerveConstants.maxTranslationalSpeed,0,0,driveBaseSubsystem.getRotation2d()));

    }
    @Override
    public void end(boolean interrupted) {
        fieldCentric.setModuleStatesFromChassisSpeed(new ChassisSpeeds(0,0,0));
        driveBaseSubsystem.brake();
    } 

    @Override
    public boolean isFinished() {
        return pitchController.atSetpoint();
    }
}
