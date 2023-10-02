// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.WristConstants;

public class WristToPosition extends CommandBase {
  private WristSubsystem wristSubsystem;
  private PIDController pidController;
  private double setpoint;

  public WristToPosition(WristSubsystem wristSubsystem, double setpoint) {
    this.wristSubsystem = wristSubsystem;
    this.setpoint = setpoint;
    SmartDashboard.putNumber("Wrist setpoint", this.setpoint);
    SmartDashboard.putNumber("Wrist kP", WristConstants.kP);
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    double kP = WristConstants.kP;
    double kD = WristConstants.kD;
    kP = SmartDashboard.getNumber("Wrist kP", kP);
    pidController = new PIDController(kP, 0, kD);
    pidController.setTolerance(WristConstants.kTolerance);
    setpoint = SmartDashboard.getNumber("Wrist Setpoint", setpoint);
    wristSubsystem.coast();
  }

  @Override
  public void execute() {
    double output = pidController.calculate(wristSubsystem.getPosition());
    wristSubsystem.setPower(output);
    SmartDashboard.putNumber("Wrist PID Output", output);
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setPower(0);
    wristSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  

}
