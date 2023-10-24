// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PowerConstants;

public class RunArmOuttake extends CommandBase {

  private ArmIntakeSubsystem armIntakeSubsystem;
  
  public RunArmOuttake(ArmIntakeSubsystem armIntakeSubsystem) {
    this.armIntakeSubsystem = armIntakeSubsystem;
    addRequirements(armIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armIntakeSubsystem.coast();
    armIntakeSubsystem.setVoltage(PowerConstants.ArmOuttakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armIntakeSubsystem.coast();
    armIntakeSubsystem.setSpeed(PowerConstants.ArmIntakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armIntakeSubsystem.setVoltage(0);
    armIntakeSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
