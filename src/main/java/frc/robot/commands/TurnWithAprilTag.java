// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;

import org.apache.commons.io.filefilter.TrueFileFilter;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TurnWithAprilTag extends CommandBase {
  
  private XboxController joystick;
  private VisionSubsystem visionSubsystem;
  private DriveBaseSubsystem driveBaseSubsystem;
  private GyroSubsystem gyroSubsystem;
  private PIDController pidController;
  // private double tolerance;
  // private double target;
  // private double initAngle;
  private double turnWithAprilTagkP;
  private double turnWithAprilTagkI;
  private double turnWithAprilTagkD;
  
  private double pidOutput;


  public TurnWithAprilTag(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, VisionSubsystem visionSubsystem, 
    double turnWithAprilTagkP, double turnWithAprilTagkI, double turnWithAprilTagkD) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.visionSubsystem = visionSubsystem;
    // this.gyroSubsystem = gyroSubsystem;
    // this.tolerance = tolerance;
    // this.target = target;
    this.turnWithAprilTagkP = turnWithAprilTagkP;
    this.turnWithAprilTagkI = turnWithAprilTagkI;
    this.turnWithAprilTagkD = turnWithAprilTagkD;
    
    addRequirements(driveBaseSubsystem);
    addRequirements(visionSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveBaseSubsystem.coast();
    pidController = new PIDController(PIDConstants.turnWithAprilTagkP, 
        PIDConstants.turnWithAprilTagkI, PIDConstants.turnWithAprilTagkD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getAButton()) {
      if (visionSubsystem.hasTarget()) {
        pidOutput = pidController.calculate(visionSubsystem.getYaw());
        driveBaseSubsystem.setLeftPower(pidOutput);
        driveBaseSubsystem.setRightPower(-pidOutput);
      }
    }

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
    return false;
  }
}
