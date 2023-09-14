// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
public class DriveBaseSubsystem extends SubsystemBase {
  /** Creates a new DriveBaseSubsystem2. */
  private CANVenom left1, left2, right1, right2;
  private final SwerveModule[] swerveModules;
  

  public DriveBaseSubsystem() {
    // left1 = new CANVenom(1);
    // left2 = new CANVenom(2);
    // right1 = new CANVenom(3);
    // right2 = new CANVenom(4);

    right2 = new CANVenom(1);
    right1 = new CANVenom(2);
    left2 = new CANVenom(3);
    left1 = new CANVenom(4);
  }
  // public DriveBaseSubsystem() {
  //   left1.setInverted(false);
  //   left2.setInverted(false);
  //   right1.setInverted(true);
  //   right2.setInverted(true);

  //   left2.follow(left1);
  //   right2.follow(right1);

  //   resetPositionAll();
  //   //voltage saturation not a thing

  // } 

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

  public void setLeftVoltage(double voltage) {
    left1.setCommand(ControlMode.VoltageControl ,voltage);
    left2.setCommand(ControlMode.VoltageControl ,voltage);
  }

  public void setRightVoltage(double voltage) {
    left1.setCommand(ControlMode.VoltageControl ,voltage);
    left2.setCommand(ControlMode.VoltageControl ,voltage);
  }

  public void setAllVoltage(double voltage) { //proportional -1 to 1
    setLeftVoltage(voltage);
    setRightVoltage(voltage);
  }

  public void setLeftPower(double power) {
    left1.setCommand(ControlMode.Proportional ,power);
    left2.setCommand(ControlMode.Proportional ,power);
  }

  public void setRightPower(double power) {
    right1.setCommand(ControlMode.Proportional ,power);
    right2.setCommand(ControlMode.Proportional ,power);
  }

  public void setAllPower(double power) {
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

  public double getDisplacementMeters(CANVenom motor) {
    return (motor.getPosition() / Constants.GearConstants.ToughboxMiniRatio * Constants.RobotConstants.kWheelCircumference); 
  }

  public double getLeftDistance() {
    return getDisplacementMeters(left1);
  }

  public double getRightDistance() {
    return getDisplacementMeters(right1);
  }
  public SwerveModule getSwerveModule(int index) {
    return swerveModules[index];
  }
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveConstants.swerve0.location, SwerveConstants.swerve1.location, SwerveConstants.swerve2.location, SwerveConstants.swerve3.location); 
    return m_kinematics;
  }

  public SwerveModule[] getSwerveModules() {
    return swerveModules;
  }
  swerveModules = new SwerveModule[] {
    new SwerveModule(SwerveConstants.swerve0.rotateMotorID, SwerveConstants.swerve0.speedMotorID, SwerveConstants.swerve0.canCoderID, 0),
    new SwerveModule(SwerveConstants.swerve1.rotateMotorID, SwerveConstants.swerve1.speedMotorID, SwerveConstants.swerve1.canCoderID, 1),
    new SwerveModule(SwerveConstants.swerve2.rotateMotorID, SwerveConstants.swerve2.speedMotorID, SwerveConstants.swerve2.canCoderID, 2),
    new SwerveModule(SwerveConstants.swerve3.rotateMotorID, SwerveConstants.swerve3.speedMotorID, SwerveConstants.swerve3.canCoderID, 3),
  };
  
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
    //putRPMOnDashBoard();
    //putPositionOnDashboard();
  }
  public void stop() {
    setAllPower(0);
  }
}
