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


public class StraightMotionMagic extends CommandBase {
  private final double leftPower = -0.25;
  private final double rightPower = -0.25;
  EncoderSubsystem Es = new EncoderSubsystem();
  public double setpoint;
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  /** Creates a new StraightMotionMagic. */
  public StraightMotionMagic(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    while(Es.encoder.getDistance()<setpoint){
      driveBaseSubsystem.setLeftPower(leftPower);
    driveBaseSubsystem.setLeftPower(rightPower);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
