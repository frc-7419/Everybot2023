// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  private TalonFX wrist;
  
  public WristSubsystem() {
    wrist = new TalonFX(Constants.WristConstants.wristCanID);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", getPosition());
  }

  public void setPower(double power) {
    wrist.set(ControlMode.PercentOutput, power);
  }

  public void brake() {
    wrist.setNeutralMode(NeutralMode.Brake);
  }

  public void coast() {
    wrist.setNeutralMode(NeutralMode.Coast);
  }

  public double getPosition() {
    return wrist.getSelectedSensorPosition() * 2 * Math.PI;
  }

  public void stop() {
    setPower(0);
  }

}
