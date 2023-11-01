// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  private TalonFX wristMotor;
  
  public WristSubsystem() {
    wristMotor = new TalonFX(Constants.CanIds.wrist.id);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40;
    config.supplyCurrLimit.triggerThresholdTime = 1.5;
    config.supplyCurrLimit.currentLimit = 30;
    wristMotor.configAllSettings(config);

  }

  public double getPosition() {
    return wristMotor.getSelectedSensorPosition();
  }

  public void setPower(double power) {
    wristMotor.set(ControlMode.PercentOutput, power);
  }

  public void brake() {
    wristMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void coast() {
    wristMotor.setNeutralMode(NeutralMode.Coast);
  }

  public boolean isOverheating() {
    return wristMotor.getTemperature()>40;
  }

  @Override
  public void periodic() {
    
  }
}
