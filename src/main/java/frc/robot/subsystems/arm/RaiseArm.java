// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class RaiseArm extends CommandBase {
  private ArmSubsystem armSubsystem;
  public RaiseArm(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.coast();
    armSubsystem.setPower(Constants.PowerConstants.ArmPower);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    armSubsystem.coast();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
