// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.CanIds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX arm; 
  public ArmSubsystem() {
    //CANID needs to be found and added
    arm = new TalonFX(CanIds.leftFalcon1.id);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPower(double power) {
    arm.set(ControlMode.PercentOutput, power);
  }

  public void setVoltage(double ticks) {
    arm.set(ControlMode.Position, ticks);
  }

  public void coast() {
    arm.setNeutralMode(NeutralMode.Coast);
  }

  public void brake() {
    arm.setNeutralMode(NeutralMode.Brake);
  }
}
