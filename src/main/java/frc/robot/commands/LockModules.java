// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;

public class LockModules extends CommandBase {
  DriveBaseSubsystem driveBaseSubsystem;
  SwerveDriveFieldCentric swerveDriveFieldCentric;
  public LockModules(DriveBaseSubsystem driveBaseSubsystem, SwerveDriveFieldCentric swerveDriveFieldCentric) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.swerveDriveFieldCentric = swerveDriveFieldCentric;
    addRequirements(driveBaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveBaseSubsystem.brake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDriveFieldCentric.setModuleStatesFromChassisSpeed(new ChassisSpeeds(0.0, 0.0, 0.0001));
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
