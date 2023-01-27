// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;



import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class GyroSubsystem extends SubsystemBase {


  public GyroSubsystem() {
    SmartDashboard.putString("subsystem", "init gyro sub");
  }
  AHRS ahrs = new AHRS();  
  @Override
  public void periodic() {
    SmartDashboard.putNumber(   "Yaw", ahrs.getYaw());
    SmartDashboard.putNumber(   "Pitch", ahrs.getPitch());
    SmartDashboard.putNumber(   "Roll", ahrs.getRoll());
  }
  public double getAngle(){
    return ahrs.getAngle();
}

  public double getYaw(){
    return ahrs.getYaw();
  }
  
  public double getPitch(){
    return ahrs.getPitch();
  }
  public double getRoll(){
    return ahrs.getRoll();
  }

}
