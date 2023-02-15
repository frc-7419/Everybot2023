// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.drive.CanVenomDriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;

import frc.robot.Constants.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TurnWithAprilTag extends CommandBase {
  
  
  private VisionSubsystem visionSubsystem;
  private CanVenomDriveBaseSubsystem canVenomDriveBaseSubsystem;
  private PIDController pidController;
  // private double tolerance;
  // private double target;
  // private double initAngle;
  private double pidOutput;


  public TurnWithAprilTag(CanVenomDriveBaseSubsystem canVenomDriveBaseSubsystem, GyroSubsystem gyroSubsystem, VisionSubsystem visionSubsystem) {
    this.canVenomDriveBaseSubsystem = canVenomDriveBaseSubsystem;
    this.visionSubsystem = visionSubsystem;
    // this.gyroSubsystem = gyroSubsystem;
    // this.tolerance = tolerance;
    // this.target = target;
    addRequirements(canVenomDriveBaseSubsystem,visionSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }


// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    canVenomDriveBaseSubsystem.coast();
    pidController = new PIDController(PIDConstants.turnWithAprilTagkP, 
        PIDConstants.turnWithAprilTagkI, PIDConstants.turnWithAprilTagkD);
    pidController.setTolerance(PIDConstants.turnWithAprilTagTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (visionSubsystem.hasTarget()) {
      pidOutput = pidController.calculate(visionSubsystem.getYaw());
      canVenomDriveBaseSubsystem.setLeftPower(pidOutput);
      canVenomDriveBaseSubsystem.setRightPower(-pidOutput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    canVenomDriveBaseSubsystem.setAllPower(0);
    canVenomDriveBaseSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
