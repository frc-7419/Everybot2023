// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunWrist extends CommandBase {
  private WristSubsystem wristSubsystem;
  private PIDController pidController;
  private TrapezoidProfile currentProfile;
  public double setpoint;
  public double tolerance = 0.01;

  
  public RunWrist(WristSubsystem wristSubsystem, double setpoint) {
    this.pidController = new PIDController(1,0,0);
    this.wristSubsystem = wristSubsystem;
    this.setpoint = setpoint;
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    wristSubsystem.setGoal(setpoint);
    double armPosition = wristSubsystem.getPosition();
    pidController.setTolerance(tolerance);
    pidController.setSetpoint(setpoint);
    wristSubsystem.coast();
  }

  @Override
  public void execute() {
    currentProfile = new TrapezoidProfile(wristSubsystem.getConstraints(), wristSubsystem.getGoal(),wristSubsystem.getStart());

    TrapezoidProfile.State nextSetpoint = currentProfile.calculate(0.02);
    wristSubsystem.setSetpoint(nextSetpoint);
    pidController.setSetpoint(nextSetpoint.position);
    double position = wristSubsystem.getPosition();
    double output = pidController.calculate(position);

    wristSubsystem.setPower(output);
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setPower(0);
    wristSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
