// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.fasterxml.jackson.databind.deser.impl.SetterlessProperty;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.LowerArm;
import frc.robot.subsystems.arm.RaiseArm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.encoder.EncoderSubsystem;


public class MoveToSetpoint extends CommandBase {
  private double leftPower;
  private double rightPower;
  private EncoderSubsystem Es;
  public double setpoint;
  private DriveBaseSubsystem driveBaseSubsystem ;
  /** Creates a new StraightMotionMagic. */
  public MoveToSetpoint(DriveBaseSubsystem driveBaseSubsystem,double setpoint, EncoderSubsystem Es) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setpoint = setpoint;
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.Es = Es;
    leftPower = -0.25;
    rightPower = -0.25;
    Es = new EncoderSubsystem();
    
    addRequirements(driveBaseSubsystem, Es);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(Es.encoder.getDistance()<setpoint){
      driveBaseSubsystem.setLeftPower(leftPower);
    driveBaseSubsystem.setLeftPower(rightPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
