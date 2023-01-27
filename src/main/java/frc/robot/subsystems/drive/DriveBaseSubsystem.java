// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class DriveBaseSubsystem extends SubsystemBase {
  /** Creates a new DriveBaseSubsystem2. */
  private CANVenom left1, left2, right1, right2;

  public DriveBaseSubsystem() {
    left1 = new CANVenom(0);
    left2 = new CANVenom(0);
    right1 = new CANVenom(0);
    right2 = new CANVenom(0);

    left1.setInverted(false);
    left2.setInverted(false);
    right1.setInverted(true);
    right2.setInverted(true);

    left2.follow(left1);
    right2.follow(right1);

    resetPositionAll();

    //voltage saturation not a thing
  } 

  public CANVenom getLeftMast(){return left1;}
  public CANVenom getRightMast(){return right1;}
  public CANVenom getLeftFollow(){return left2;}
  public CANVenom getRightFollow(){return right2;}

  public void setAllControlMode(CANVenom.ControlMode mode) {
    left1.setControlMode(mode);
    left2.setControlMode(mode);
    right1.setControlMode(mode);
    right2.setControlMode(mode);
  }

  public void setAllBrakeCoastMode(CANVenom.BrakeCoastMode mode) {
    left1.setBrakeCoastMode(mode);
    left2.setBrakeCoastMode(mode);
    right1.setBrakeCoastMode(mode);
    right2.setBrakeCoastMode(mode);
  }

  public void coast() {
    setAllBrakeCoastMode(BrakeCoastMode.Coast);
  }

  public void brake() {
    setAllBrakeCoastMode(BrakeCoastMode.Brake);
  }

  public void setLeftPower(double power) {
    left1.setCommand(ControlMode.Proportional ,power);
    left2.setCommand(ControlMode.Proportional ,power);
  }

  public void setRightPower(double power) {
    right1.setCommand(ControlMode.Proportional ,power);
    right2.setCommand(ControlMode.Proportional ,power);
  }

  public void setAllPower(double power) { //proportional -1 to 1
    left1.setCommand(ControlMode.Proportional ,power);
    left2.setCommand(ControlMode.Proportional ,power);
    right1.setCommand(ControlMode.Proportional ,power);
    right2.setCommand(ControlMode.Proportional ,power);
  }

  public void resetPositionAll() {
    left1.resetPosition();
    left2.resetPosition();
    right1.resetPosition();
    right2.resetPosition();
  }

  public void putRPMOnDashBoard() {
    SmartDashboard.putNumber("Left Mast RPM", left1.getSpeed());
    SmartDashboard.putNumber("Left Follow RPM", left2.getSpeed());
    SmartDashboard.putNumber("Right Mast RPM", right1.getSpeed());
    SmartDashboard.putNumber("Right Follow RPM", right2.getSpeed());
  }
  
  public void putPositionOnDashboard() {
    SmartDashboard.putNumber("Left Mast Revolutions ", left1.getPosition());
    SmartDashboard.putNumber("Left Follow Revolutions", left2.getPosition());
    SmartDashboard.putNumber("Right Mast Revolutions", right1.getPosition());
    SmartDashboard.putNumber("Right Follow Revolutions", right2.getPosition()); 

    SmartDashboard.putNumber("Left Mast Position (m) ", getPositionMeters(left1));
    SmartDashboard.putNumber("Left Follow Position (m)", getPositionMeters(left2));
    SmartDashboard.putNumber("Right Mast Position (m)", getPositionMeters(right1));
    SmartDashboard.putNumber("Right Follow Position (m)", getPositionMeters(right2)); 
  }

  public double getPositionMeters(CANVenom motor) {
    return motor.getPosition() * Constants.RobotConstants.kWheelCircumference;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putRPMOnDashBoard();
    putPositionOnDashboard();
  }
}


//all the old code 7419 copied to this repo (since they used talon srx controllers), which is not what we will be using

/*
 * package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class DriveBaseSubsystem extends SubsystemBase {
    public TalonSRX left1;
    public TalonSRX right1;
	  public TalonSRX left2;
    public TalonSRX right2;
  
  public DriveBaseSubsystem() {
    left1 = new TalonSRX(CanIds.leftFalcon1.id);
	  right1 = new TalonSRX(CanIds.rightFalcon1.id);
	  left2 = new TalonSRX(CanIds.leftFalcon2.id);
    right2 = new TalonSRX(CanIds.rightFalcon2.id);

    factoryResetAll();

    right1.setInverted(true);
    right1.setSensorPhase(false);
    right2.setInverted(true);
    right2.setSensorPhase(false);

    left1.setInverted(false);
    left2.setInverted(false);

    left2.follow(left1);
    right2.follow(right1);

    //comment out lines 35-46 in case turns are off and there is no time to tune
    left1.configVoltageCompSaturation(11);
    left1.enableVoltageCompensation(true);

    left2.configVoltageCompSaturation(11);
    left2.enableVoltageCompensation(true);

    right1.configVoltageCompSaturation(11);
    right2.enableVoltageCompensation(true);

    right2.configVoltageCompSaturation(11);
    right2.enableVoltageCompensation(true);
  }

  @Override
  public void periodic() {
  }

  public enum TurnDirection {
    LEFT,
    RIGHT,
  }

  // accessors
  public TalonSRX getLeftMast(){return left1;}
  public TalonSRX getRightMast(){return right1;}
  public TalonSRX getLeftFollow(){return left2;}
  public TalonSRX getRightFollow(){return right2;}

  public void setLeftVoltage(double voltage){ //comment this method out as well
    left1.set(ControlMode.PercentOutput, voltage/11);
    left2.set(ControlMode.PercentOutput, voltage/11);
  }

  public void setRightVoltage(double voltage){ //comment this method out as well
    right1.set(ControlMode.PercentOutput, voltage/11);
    right2.set(ControlMode.PercentOutput, voltage/11);
  }

  public void setAllVoltage(double voltage){ //comment this method out as well
    setLeftVoltage(voltage);
    setRightVoltage(voltage);
  }

  public void setLeftPower(double power){
    left1.set(ControlMode.PercentOutput, power);
    left2.set(ControlMode.PercentOutput, power);
  }

  public void setRightPower(double power){
    right1.set(ControlMode.PercentOutput, power);
    right2.set(ControlMode.PercentOutput, power);
  }

  public void setAllPower(double power){
    setLeftPower(power);
    setRightPower(power);
  }

  public void stop(){setAllPower(0);}

  public void setAllMode(NeutralMode mode){
    right1.setNeutralMode(mode);
    right2.setNeutralMode(mode);
    left1.setNeutralMode(mode);
    left2.setNeutralMode(mode);
  }

  public void brake(){setAllMode(NeutralMode.Brake);}

  public void coast(){setAllMode(NeutralMode.Coast);}

  public double getLeftVelocity(){return left1.getSelectedSensorVelocity();}
  public double getRightVelocity(){return right1.getSelectedSensorVelocity();}

  public void setAllDefaultInversions() {
    right1.setInverted(true);
    right2.setInverted(true);
    left1.setInverted(false);
    left2.setInverted(false);
  }

  public void factoryResetAll() {
    right1.configFactoryDefault();
    right2.configFactoryDefault();
    left1.configFactoryDefault();
    left2.configFactoryDefault();
  }
}

 */