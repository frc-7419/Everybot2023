// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkMax arm;
  public ArmSubsystem() {
    //CANID needs to be found and added
    arm = new CANSparkMax(0, MotorType.kBrushless); //Neo 550 (Brushless) is what Robonauts use, maybe we will use a different motor
    //perhaps think of having 11 V voltage compensation in future as 7419 does
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPower(double power) {
    arm.set(power);
  }

  public void setVoltage(double voltage) {
    arm.setVoltage(voltage);
  }

  public void coast() {
    arm.setIdleMode(IdleMode.kCoast);
  }

  public void brake() {
    arm.setIdleMode(IdleMode.kBrake);
  }
}
