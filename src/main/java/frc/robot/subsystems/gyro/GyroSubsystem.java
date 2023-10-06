// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;



import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class GyroSubsystem extends SubsystemBase {

  AHRS ahrs;

  public GyroSubsystem() {
    try { 
      /*
       * Communicate w/navX-MXP via the MXP SPI Bus (use mini USB to USB A cable)
       * Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or S
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      ahrs = new AHRS(SerialPort.Port.kMXP);
      SmartDashboard.putString("subsystem", "init gyro sub");
      ahrs.zeroYaw(); //field centric, we need yaw to be zero
  } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
  }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(   "Yaw", ahrs.getYaw());
    SmartDashboard.putNumber(   "Pitch", ahrs.getPitch());
    SmartDashboard.putNumber(   "Roll", ahrs.getRoll());
  }

  public double getAngle(){
      return ahrs.getAngle();
  }

  public double getYaw() {
    return ahrs.getYaw();
  }

  public double getPitch() {
    return ahrs.getPitch();
  }
  public double getRoll() {
    return ahrs.getRoll();
  }

  public void resetYaw() {
    ahrs.reset();
    ahrs.zeroYaw();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(ahrs.getYaw());
    /*the thing is .getYaw is -180 to 180 so it not being 0 to 360 
    may cause the internal conversion that Rotation2d does to be wrong 
    */
  }
}