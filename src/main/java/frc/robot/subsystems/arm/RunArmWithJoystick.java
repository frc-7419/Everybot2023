// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PowerConstants;

public class RunArmWithJoystick extends CommandBase {
  private final XboxController joystick;
  private final ArmSubsystem armSubsystem;
  public RunArmWithJoystick(XboxController joystick, ArmSubsystem armSubsystem) {
    this.joystick = joystick;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.coast(); //not sure why coast at init but 7419 did so for elevator
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(joystick.getLeftY()) > 0.05) {
      armSubsystem.coast();
      double power = joystick.getLeftY() * PowerConstants.maxArmPower;
      armSubsystem.setPower(power);
      
      SmartDashboard.putNumber("Arm Power", power);
    }
    else {
      //TODO: Replace the 0 with some tested constant that keeps the arm stationary - PowerConstants.armStationaryPower
      armSubsystem.setPower(0);
      armSubsystem.brake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setPower(0);
    armSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}