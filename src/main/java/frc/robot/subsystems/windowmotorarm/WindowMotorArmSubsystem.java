// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.windowmotorarm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WindowMotorArmSubsystem extends SubsystemBase {
  private TalonFX armMotor;
  public WindowMotorArmSubsystem() {
    this.armMotor = new TalonFX(00); //temporary CAN id
  }

  @Override
  public void periodic() {
    
  }
  public void setPower(double power) {
    armMotor.set(ControlMode.PercentOutput, power);
  }

  public void coast() {
    armMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void brake() {
    armMotor.setNeutralMode(NeutralMode.Brake);
  }
}
