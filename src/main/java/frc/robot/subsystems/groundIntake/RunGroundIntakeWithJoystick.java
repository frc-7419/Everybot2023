// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunGroundIntakeWithJoystick extends CommandBase {
  private GroundIntakeSubsystem groundIntakeSubsystem;
  private XboxController joystick;
  public RunGroundIntakeWithJoystick(GroundIntakeSubsystem groundIntakeSubsystem, XboxController joystick) {
    this.groundIntakeSubsystem = groundIntakeSubsystem;
    this.joystick = joystick;
    addRequirements(groundIntakeSubsystem);
  }
  @Override
  public void initialize() {
    groundIntakeSubsystem.coast();
  }
  @Override
  public void execute() {
    if (joystick.getLeftTriggerAxis()>0.5) {
      groundIntakeSubsystem.coast();
      groundIntakeSubsystem.setSpeed(-0.5);
    } else if (joystick.getRightTriggerAxis()>0.5) {
      groundIntakeSubsystem.coast();
      groundIntakeSubsystem.setSpeed(0.5);
    } else {
      groundIntakeSubsystem.setSpeed(0);
      groundIntakeSubsystem.brake();
    }
  }
  @Override
  public void end(boolean interrupted) {
    groundIntakeSubsystem.setSpeed(0);
    groundIntakeSubsystem.brake();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}
