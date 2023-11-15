// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveFieldCentric extends CommandBase {
  private XboxController joystick;
  private DriveBaseSubsystem driveBaseSubsystem;

  public SwerveDriveFieldCentric(XboxController joystick, DriveBaseSubsystem driveBaseSubsystem) {
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
    addRequirements(driveBaseSubsystem);
  }
  /**
   * Sets the individual swerve module states
   * @param moduleStates
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    for (int i=0; i<4; ++i) {
      driveBaseSubsystem.getSwerveModule(i).setSwerveModuleState(moduleStates[i]);
    }
  }
  public void setModuleStatesTeleop(SwerveModuleState[] moduleStates, XboxController joystick) {
    for (int i=0; i<4; ++i) {
      driveBaseSubsystem.getSwerveModule(i).setSwerveModuleState(moduleStates[i], joystick);
    }
  }
  /**
   * Sets the module states directly from the chassis speed
   * @param chassisSpeeds
   */
  public void setModuleStatesFromChassisSpeed(ChassisSpeeds chassisSpeeds) {
    setModuleStates(driveBaseSubsystem.ChassisSpeedstoModuleSpeeds(chassisSpeeds));
  }
  public void setModuleStatesFromChassisSpeedTeleop(ChassisSpeeds chassisSpeeds, XboxController joystick) {
    setModuleStatesTeleop(driveBaseSubsystem.ChassisSpeedstoModuleSpeeds(chassisSpeeds), joystick);
  }
  /**
   * this is what makes the robot begin moving, the entry point for swerve centric drive!
   * @param joystick
   */
  public void setModuleStatesFromJoystick(XboxController joystick) {
    setModuleStatesFromChassisSpeedTeleop(driveBaseSubsystem.getChassisSpeedsFromJoystick(joystick), joystick);
  }

  @Override
  public void initialize() {
    driveBaseSubsystem.coast();
    driveBaseSubsystem.zeroYaw();
  }

  @Override
  public void execute() {
    setModuleStatesFromJoystick(joystick);
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}