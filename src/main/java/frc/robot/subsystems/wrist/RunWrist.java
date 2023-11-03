// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunWrist extends CommandBase {
  private WristSubsystem wristSubsystem;
  private double setpoint;
  public RunWrist(WristSubsystem wristSubsystem, double setpoint) {
    this.wristSubsystem = wristSubsystem;
    this.setpoint = setpoint;
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    wristSubsystem.coast();
  }

  @Override
  public void execute() {
    wristSubsystem.setPower(wristSubsystem.getPosition()-setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setPower(0);
    wristSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(wristSubsystem.getPosition()-setpoint)<2;
  }
}
