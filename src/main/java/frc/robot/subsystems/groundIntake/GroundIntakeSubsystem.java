// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GroundIntakeSubsystem extends SubsystemBase {
  TalonSRX leftMotor;
  TalonSRX rightMotor;
  public GroundIntakeSubsystem() {
    this.leftMotor = new TalonSRX(Constants.CanIds.leftgroundintake.id);
    this.rightMotor = new TalonSRX(Constants.CanIds.rightgroundintake.id);
  }
  public void setSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }
  public void coast() {
    leftMotor.setNeutralMode(NeutralMode.Coast);
    rightMotor.setNeutralMode(NeutralMode.Coast);
  }
  public void brake() {
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
  }
  

  @Override
  public void periodic() {}
}
