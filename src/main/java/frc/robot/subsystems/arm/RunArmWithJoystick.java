// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunArmWithJoystick extends CommandBase {
  private final XboxController joystick;
  private final ArmSubsystem armSubsystem;
  public RunArmWithJoystick(XboxController joystick, ArmSubsystem armSubsystem) {
    this.joystick = joystick;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.coast(); //not sure why coast at init but 7419 did so for elevator
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Arm Percent Power", armSubsystem.getPercentPower());
    if (joystick.getLeftY() != 0) {
      armSubsystem.coast();
      armSubsystem.setPower(joystick.getLeftY() * 0.1);
    }
    else {
      armSubsystem.setPower(0);
      armSubsystem.brake();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
