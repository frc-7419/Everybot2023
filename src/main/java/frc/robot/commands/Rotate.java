// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;

public class Rotate extends CommandBase {
  private double degrees;
  private DriveBaseSubsystem driveBaseSubsystem;
  private SwerveDriveFieldCentric swerveDriveFieldCentric;
  private double startingYaw;

  public Rotate(DriveBaseSubsystem driveBaseSubsystem, SwerveDriveFieldCentric swerveDriveFieldCentric,double degrees) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.swerveDriveFieldCentric = swerveDriveFieldCentric;
    this.degrees = degrees;
    addRequirements(driveBaseSubsystem);
  }

  @Override
  public void initialize() {
    //add 180 to yaw because I said so
    startingYaw = driveBaseSubsystem.getYaw() +180;
    driveBaseSubsystem.coast();
  }

  @Override
  public void execute() {
    double currentYaw = driveBaseSubsystem.getYaw()+180;
    double deltaYaw = currentYaw-degrees;
    // if(deltaYaw > degrees) {
    //   swerveDriveFieldCentric.setModuleStatesFromChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0,0, Math.PI/2,driveBaseSubsystem.getRotation2d()));
    // }
    // else {
    //   swerveDriveFieldCentric.setModuleStatesFromChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0,0, Math.PI/2,driveBaseSubsystem.getRotation2d()));
    // }
    swerveDriveFieldCentric.setModuleStatesFromChassisSpeed(new ChassisSpeeds(0, 0, Math.PI/6));
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveFieldCentric.setModuleStatesFromChassisSpeed(new ChassisSpeeds(0,0,0));
    driveBaseSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(degrees-(driveBaseSubsystem.getYaw()+320.4))<3;
  }
}
