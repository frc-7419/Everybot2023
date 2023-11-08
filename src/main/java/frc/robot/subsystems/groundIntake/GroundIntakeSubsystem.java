// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.WristConstants;

public class GroundIntakeSubsystem extends SubsystemBase {
  //wrist vars
  private TalonFX wristMotor;
  //TODO: find the correct encoder value
  private final TrapezoidProfile.Constraints constraints;
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(CanIds.wrist.id);
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State start;  

  private TalonSRX leftMotor;
  private TalonSRX rightMotor;
  public GroundIntakeSubsystem() {
    this.leftMotor = new TalonSRX(Constants.CanIds.leftgroundintake.id);
    this.rightMotor = new TalonSRX(Constants.CanIds.rightgroundintake.id);

    //wrist configs
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
  public void setIntakeSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }
  public void intakeCoast() {
    leftMotor.setNeutralMode(NeutralMode.Coast);
    rightMotor.setNeutralMode(NeutralMode.Coast);
  }
  public void intakeBrake() {
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
  }

  //wrist functions

  public TrapezoidProfile.Constraints getConstraints() {
    return constraints;   
  }
  public TrapezoidProfile.State getStart() {
    return start;
  }
  public TrapezoidProfile.State getGoal() {
    return goal;
  }
  public void setWristGoal(double goal){
    this.goal = new TrapezoidProfile.State(goal, 0);
  }
  public void setWristSetpoint(TrapezoidProfile.State nextSetpoint) {
    start = nextSetpoint;
  }

  public void setWristPower(double power) {
    wristMotor.set(ControlMode.PercentOutput, power);
  }

  public void wristBrake() {
    wristMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void wristCoast() {
    wristMotor.setNeutralMode(NeutralMode.Coast);
  }

  public boolean isWristOverheating() {
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
    SmartDashboard.putNumber("Absolute Wrist Encoder Angle", getPosition());
    SmartDashboard.putBoolean("connected", encoder.isConnected());
  }
}
