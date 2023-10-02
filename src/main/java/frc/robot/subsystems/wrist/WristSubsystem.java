// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  private CANSparkMax wrist;
  
  public WristSubsystem() {
    wrist = new CANSparkMax(Constants.CanIds.wrist.id, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", getPosition());
  }

  public void setPower(double power) {
    wrist.set(power);
  }

  public void brake() {
    wrist.setIdleMode(IdleMode.kBrake);
  }

  public void coast() {
    wrist.setIdleMode(IdleMode.kCoast);
  }

  public double getPower() {
    return wrist.get();
  }

  public double getPosition() {
    return wrist.getEncoder().getPosition() * 2 * Math.PI;
  }

  public void stop() {
    setPower(0);
  }

}
