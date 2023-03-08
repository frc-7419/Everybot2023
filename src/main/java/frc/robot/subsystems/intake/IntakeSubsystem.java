// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intake;
  public IntakeSubsystem() {
    intake = new CANSparkMax(11, MotorType.kBrushless); //Neo 550 probably
    // intake.enableVoltageCompensation(11);
  }

  @Override
  public void periodic() {}

  public void setPower(double power) {
    intake.set(power);
  }

  public void brake() {
    intake.setIdleMode(IdleMode.kBrake);
  }

  public void coast() {
    intake.setIdleMode(IdleMode.kCoast);
  }

  public double getPower() {
    return intake.get();
  }
}
