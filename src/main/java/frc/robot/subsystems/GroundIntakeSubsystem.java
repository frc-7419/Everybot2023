// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeSubsystem extends SubsystemBase {
  /** Creates a new groundIntakeSubsystem. */

  private TalonSRX left;
  private TalonSRX right;
  
  public GroundIntakeSubsystem() {
    //replace with constants later
    left = new TalonSRX(Constants.GroundIntakeConstants.leftIntakeCanID);
    right = new TalonSRX(Constants.GroundIntakeConstants.rightIntakeCanID);

    left.configFactoryDefault();
    right.configFactoryDefault();
    left.setInverted(true);
    right.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRightPower(double power){
    right.set(ControlMode.PercentOutput, power);
  }
  public void setLeftPower(double power){
    left.set(ControlMode.PercentOutput, power);
  }
  public void setAllPower(double power){
    setRightPower(power);
    setLeftPower(power);
  }
  public double getLeftVelocity(){
    return left.getSelectedSensorVelocity();
  }
  public double getRightVelocity(){
    return right.getSelectedSensorVelocity();
  }
  public void coast(){
    left.setNeutralMode(NeutralMode.Coast);
    right.setNeutralMode(NeutralMode.Coast);
  }
  public void brake(){
    left.setNeutralMode(NeutralMode.Brake);
    right.setNeutralMode(NeutralMode.Brake);
  }

  public void runGroundIntake(double power) {
    coast();
    setAllPower(power);
  }
}
