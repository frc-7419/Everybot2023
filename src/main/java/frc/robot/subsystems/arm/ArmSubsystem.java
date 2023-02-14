// Copyright (c) FIRST and other WPILib contributors.
package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class ArmSubsystem extends SubsystemBase {
  private TalonSRX arm;

  public ArmSubsystem() {
    arm = new TalonSRX(CanIds.arm.id);
  }

  @Override
  public void periodic() {}

  public void setPower(double power) {
    arm.set(ControlMode.PercentOutput, power);
  }

  public void coast() {
    arm.setNeutralMode(NeutralMode.Coast);
  }

  public void brake() {
    arm.setNeutralMode(NeutralMode.Brake);
  }
}
