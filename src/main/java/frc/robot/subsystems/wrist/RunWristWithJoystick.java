// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.PowerConstants;

public class RunWristWithJoystick extends CommandBase {
  private WristSubsystem wristSubsystem;
  private XboxController joystick;
  
  public RunWristWithJoystick(WristSubsystem wristSubsystem, XboxController joystick) {
    this.wristSubsystem = wristSubsystem;
    this.joystick = joystick;
  }

  @Override
  public void initialize() {
    wristSubsystem.coast();
  }

  @Override
  public void execute() {
    double power = PowerConstants.WristPower;
    if (joystick.getLeftBumper()) {
      wristSubsystem.coast();
      wristSubsystem.setPower(-power);
    } else if (joystick.getRightBumper()) {
      wristSubsystem.coast();
      wristSubsystem.setPower(power);
    } else {
      wristSubsystem.brake();
      wristSubsystem.setPower(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stop();
    wristSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  

}
