// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.PIDConstants.*;
import static frc.robot.Constants.RobotConstants.*;

public class ArmWithPID extends CommandBase {
  /** Creates a new ArmWithPID2. */
  ArmSubsystem armSubsystem;
  PIDController positionController;
  double setpoint;
  double tolerance;
  public ArmWithPID(ArmSubsystem armSubsystem, double setpoint) {
    this.armSubsystem = new ArmSubsystem();
    this.positionController = new PIDController(ArmAngleKp, ArmAngleKi, ArmAngleKd);
    this.setpoint = setpoint;
    this.tolerance = 0.5;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.coast();
    positionController.setTolerance(tolerance);
    positionController.setSetpoint(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double position = armSubsystem.getPosition() - armEncoderOffset;
    double output = positionController.calculate(position);
    SmartDashboard.putNumber("arm output", output);
    // armSubsystem.setPower(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return positionController.atSetpoint();
  }
}
