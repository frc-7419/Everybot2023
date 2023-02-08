// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkMax arm;
  private PigeonIMU pigeon;
  private TalonSRX talon;
  public ArmSubsystem() {
    //CANID needs to be found and added
    arm = new CANSparkMax(3, MotorType.kBrushless); //Neo 550 (Brushless) is what Robonauts use, maybe we will use a different motor
    //perhaps think of having 11 V voltage compensation in future as 7419 does
    talon = new TalonSRX(51);
    pigeon = new PigeonIMU(talon);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("yaw", pigeon.getYaw());
    // SmartDashboard.putNumber("pitch", pigeon.getPitch());
    // SmartDashboard.putNumber("roll", pigeon.getRoll());
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
