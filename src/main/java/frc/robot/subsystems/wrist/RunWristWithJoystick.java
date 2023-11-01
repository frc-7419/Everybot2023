// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunWristWithJoystick extends CommandBase {
  WristSubsystem wristSubsystem;
  XboxController joystick;
  public RunWristWithJoystick(WristSubsystem wristSubsystem, XboxController joystick) {
    this.joystick = joystick;
    this.wristSubsystem = wristSubsystem;
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    wristSubsystem.coast();
    wristSubsystem.setPower(0);
  }

  @Override
  public void execute() {
    if(joystick.getRightY()>0.07){
      wristSubsystem.coast();
      wristSubsystem.setPower(joystick.getRightY()/4);
    }
    else {
      wristSubsystem.brake();
      wristSubsystem.setPower(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.brake();
    wristSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return wristSubsystem.isOverheating();
  }
}
