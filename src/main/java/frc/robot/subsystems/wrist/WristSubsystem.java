// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
  private TalonFX wristMotor;
  //TODO: find the correct encoder value
  private final TrapezoidProfile.Constraints constraints;
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(CanIds.wrist.id);
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State start;  
  
  public WristSubsystem() {
    constraints = new TrapezoidProfile.Constraints(WristConstants.maxVelocity, WristConstants.maxAcceleration);
    start = new TrapezoidProfile.State(0, 0);
    wristMotor = new TalonFX(Constants.CanIds.wrist.id);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40;
    config.supplyCurrLimit.triggerThresholdTime = 1.5;
    config.supplyCurrLimit.currentLimit = 30;
    wristMotor.configAllSettings(config);

    //TODO: Find the correct offset
    encoder.setPositionOffset(0.144);
  }

  public TrapezoidProfile.Constraints getConstraints() {
    return constraints;   
  }
  public TrapezoidProfile.State getStart() {
    return start;
  }
  public TrapezoidProfile.State getGoal() {
    return goal;
  }
  public void setGoal(double goal){
    this.goal = new TrapezoidProfile.State(goal, 0);
  }
  public void setSetpoint(TrapezoidProfile.State nextSetpoint) {
    start = nextSetpoint;
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

  public double getPosition() {
    return encoder.getAbsolutePosition() - encoder.getPositionOffset();
  }

  public boolean isConnected() {
    return encoder.isConnected();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Absolute Encoder Angle", getPosition());
    SmartDashboard.putBoolean("connected", encoder.isConnected());
  }
}
