package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class ZeroFieldCentric extends CommandBase {

    
    private DriveBaseSubsystem driveBaseSubsystem;

    public ZeroFieldCentric(DriveBaseSubsystem driveBaseSubsystem) {
        this.driveBaseSubsystem = driveBaseSubsystem;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void initialize() {
        driveBaseSubsystem.zeroYaw();
    }

    @Override
    public void execute() {
        // driveBaseSubsystem.zeroYaw();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
