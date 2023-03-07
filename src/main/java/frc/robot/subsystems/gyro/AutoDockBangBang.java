// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PowerConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class AutoDockBangBang extends CommandBase {
  private GyroSubsystem gyroSubsystem;
  private DriveBaseSubsystem driveBaseSubsystem;
  private final double PITCH_SETPOINT = 0;

  public AutoDockBangBang(GyroSubsystem gyroSubsystem, DriveBaseSubsystem driveBaseSubsystem) {

    this.gyroSubsystem = gyroSubsystem;
    this.driveBaseSubsystem = driveBaseSubsystem;

    driveBaseSubsystem.coast();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double pitch = gyroSubsystem.getPitch();

    double output = PowerConstants.autoDockPower;

    if (pitch < output) {
      output *= -1;
    }

    driveBaseSubsystem.setAllPower(output);
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.setAllPower(0);
    driveBaseSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return gyroSubsystem.getPitch() == PITCH_SETPOINT;
  }
}
