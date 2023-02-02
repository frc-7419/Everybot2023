// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class TurnWithGyroClosedLoop extends CommandBase {
  private DriveBaseSubsystem driveBaseSubsystem;
 private  GyroSubsystem gyroSubsystem;
 private  double target;
 private  double tolerance;
 private  double kP;
 private  double kI;
 private  double kD;
 private  PIDController pidController;
 private  double pidOutput;
 private  double initAngle;

  /** Creates a new TurnWithGyroClosedLoop. */
  public TurnWithGyroClosedLoop(DriveBaseSubsystem driveBaseSubsystem,
   GyroSubsystem gyroSubsystem,
   double target,
   double tolerance,
   double kP,
   double kI,
   double kD,
   PIDController pidController,
   double pidOutput,
   double initAngle) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.gyroSubsystem = gyroSubsystem;
    this.target = target;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.tolerance = tolerance;
    this.pidOutput = pidOutput;
    this.initAngle = initAngle;
    this.pidController = pidController;


    //TODOContinue/FINISH THIS
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveBaseSubsystem.coast();
    initAngle = gyroSubsystem.getYaw();
    pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(tolerance);
    //TODOContinue/FINISH THIS
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidOutput = pidController.calculate(gyroSubsystem.getYaw());
    driveBaseSubsystem.setLeftPower(pidOutput);
    driveBaseSubsystem.setRightPower(- pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.stop(); 
    driveBaseSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
