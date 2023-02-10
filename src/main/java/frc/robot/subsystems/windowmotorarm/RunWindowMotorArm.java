// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.windowmotorarm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunWindowMotorArm extends CommandBase {
  private WindowMotorArmSubsystem windowMotorArmSubsystem;
  private double powerConstant = 0.5;
  private XboxController joystick;
  public RunWindowMotorArm(WindowMotorArmSubsystem windowMotorArmSubsystem, XboxController joystick) {
    this.windowMotorArmSubsystem = windowMotorArmSubsystem;
    this.joystick = joystick;
    addRequirements(windowMotorArmSubsystem);
  }

  @Override
  public void initialize() {
    windowMotorArmSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getLeftY() != 0) {
      windowMotorArmSubsystem.coast();
      windowMotorArmSubsystem.setPower(powerConstant * joystick.getLeftY());
    }
    else {
      windowMotorArmSubsystem.setPower(0);
      windowMotorArmSubsystem.brake();
    }
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
