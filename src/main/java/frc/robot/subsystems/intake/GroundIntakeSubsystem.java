// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.Constants;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.CanIdsNoEnum;

import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeSubsystem extends SubsystemBase {
  /** Creates a new groundIntakeSubsystem. */

  private CANSparkMax left;
  private CANSparkMax right;
  
  public GroundIntakeSubsystem() {
    //replace with constants later
    left = new CANSparkMax(CanIds.leftIntake.id, MotorType.kBrushless);
    right = new CANSparkMax(CanIds.rightIntake.id, MotorType.kBrushless);


    // left.setInverted(false);
    // right.setInverted(true);
  }

  public void setRightVoltage(double power){
    right.setVoltage(power);
  }
  public void setLeftVoltage(double power){
    //left.setVoltage(power);
  }

  public void setAllVoltage(double power) {
    setRightVoltage(power);
    //setLeftVoltage(power);
  }

  public void setRightPower(double power){
    right.set(power);
  }
  public void setLeftPower(double power){
    left.set(power);
  }
  public void setAllPower(double power){
    setRightPower(power);
    setLeftPower(power);
    
  }
  public void coast(){
    left.setIdleMode(CANSparkMax.IdleMode.kCoast);
    right.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }
  public void brake(){
    left.setIdleMode(CANSparkMax.IdleMode.kBrake);
    right.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void runIntake(double power) {
    coast();
    setAllPower(power);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void stop(){
    setAllVoltage(0);
  }
}
