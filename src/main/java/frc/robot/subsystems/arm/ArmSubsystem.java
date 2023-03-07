// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX arm; 
  public ArmSubsystem() {
    arm = new TalonFX(CanIds.leftFalcon1.id);
    arm.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPower(double power) {
    arm.set(ControlMode.PercentOutput, power);
  }

  public void coast() {
    arm.setNeutralMode(NeutralMode.Coast);
  }

  public void brake() {
    arm.setNeutralMode(NeutralMode.Brake);
  }
  
  public double getVoltage() {
    return arm.getMotorOutputVoltage();
  }

  public double getPercentPower() {
    return arm.getMotorOutputPercent();
  }
}
