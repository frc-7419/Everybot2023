// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunWrist extends CommandBase {
  private GroundIntakeSubsystem groundIntakeSubsystem;
  private PIDController pidController;
  private TrapezoidProfile currentProfile;
  public double setpoint;
  public double tolerance = 0.01;

  
  public RunWrist(GroundIntakeSubsystem groundIntakeSubsystem, double setpoint) {
    this.pidController = new PIDController(0.02,0,0);
    this.groundIntakeSubsystem = groundIntakeSubsystem;
    this.setpoint = setpoint;
    addRequirements(groundIntakeSubsystem);
  }

  @Override
  public void initialize() {
    // groundIntakeSubsystem.setWristGoal(setpoint);
    double armPosition = groundIntakeSubsystem.getPosition();
    pidController.setTolerance(tolerance);
    pidController.setSetpoint(setpoint);
    groundIntakeSubsystem.wristCoast();
  }

  @Override
  public void execute() {
    // currentProfile = new TrapezoidProfile(groundIntakeSubsystem.getConstraints(), groundIntakeSubsystem.getGoal(),groundIntakeSubsystem.getStart());

    // TrapezoidProfile.State nextSetpoint = currentProfile.calculate(0.02);
    // groundIntakeSubsystem.setWristSetpoint(setpoint);
    pidController.setSetpoint(setpoint);
    double position = groundIntakeSubsystem.getPosition();
    double output = MathUtil.clamp(pidController.calculate(position),-0.2,0.2);

    groundIntakeSubsystem.setWristPower(output);
  }

  @Override
  public void end(boolean interrupted) {
    groundIntakeSubsystem.setWristPower(0);
    // groundIntakeSubsystem.wristBrake();
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
