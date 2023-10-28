package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;

public class AutoDock extends CommandBase {
    PIDController pitchController;
    private DriveBaseSubsystem driveBaseSubsystem;
    private SwerveDriveFieldCentric fieldCentric;
    private double recordedStartingPitch;
    private boolean toggle = true;

    //constants
    private double SETPOINT = 0;
    private double TOLERANCE = 0.1;
    
    public AutoDock(DriveBaseSubsystem driveBaseSubsystem, SwerveDriveFieldCentric fieldCentric) {
        this.pitchController = new PIDController(0.01, 0, 0);
        this.fieldCentric = fieldCentric;
        this.driveBaseSubsystem = driveBaseSubsystem;
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
        if((Math.abs(driveBaseSubsystem.getPitch())-recordedStartingPitch)>10) toggle = false;
        if(toggle) {
            output = 0.4;
        }
        else {
            output = pitchController.calculate(driveBaseSubsystem.getPitch());
        }
        fieldCentric.setModuleStates(fieldCentric.ChassisSpeedstoModuleSpeeds(new ChassisSpeeds(output,0,0)));

    }
    @Override
    public void end(boolean interrupted) {
        driveBaseSubsystem.brake();
    } 

    @Override
    public boolean isFinished() {
        return pitchController.atSetpoint();
    }
}