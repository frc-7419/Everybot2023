// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBaseSubsystem extends SubsystemBase {
  /** Creates a new DriveBaseSubsystem2. */
  private SwerveModule[] swerveModules;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;
  private SwerveModulePosition[] positions;
  private AHRS ahrs;

  public DriveBaseSubsystem() {
    // remember when setting up, swerve0-3 has to be in this orientation:
    // m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
    // m_backRightLocation respectively
    swerveModules = new SwerveModule[] {
        new SwerveModule(SwerveConstants.frontLeft.turnMotorID, SwerveConstants.frontLeft.speedMotorID,
            SwerveConstants.frontLeft.turnEncoderID, SwerveConstants.frontLeft.absolutePositionAtRobotZero, 94.219, 0),
        new SwerveModule(SwerveConstants.frontRight.turnMotorID, SwerveConstants.frontRight.speedMotorID,
            SwerveConstants.frontRight.turnEncoderID, SwerveConstants.frontRight.absolutePositionAtRobotZero, 28.125,
            1),
        new SwerveModule(SwerveConstants.backRight.turnMotorID, SwerveConstants.backRight.speedMotorID,
            SwerveConstants.backRight.turnEncoderID, SwerveConstants.backRight.absolutePositionAtRobotZero, 266.572, 2),
        new SwerveModule(SwerveConstants.backLeft.turnMotorID, SwerveConstants.backLeft.speedMotorID,
            SwerveConstants.backLeft.turnEncoderID, SwerveConstants.backLeft.absolutePositionAtRobotZero, 177, 3),
    };
    ahrs = new AHRS(SerialPort.Port.kMXP);
    ahrs.zeroYaw(); // field centric, we need yaw to be zero

    m_kinematics = new SwerveDriveKinematics(SwerveConstants.frontLeft.location, SwerveConstants.frontRight.location,
        SwerveConstants.backRight.location, SwerveConstants.backLeft.location);
    
    coast();
  }

  public void zeroYaw() {
    ahrs.zeroYaw();
  }

  public SwerveModule getSwerveModule(int index) {
    return swerveModules[index];
  }

  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return m_kinematics;
  }

  public SwerveModule[] getSwerveModules() {
    return swerveModules;
  }

  public double getYaw() { // CW IS POSITIVE BY DEFAULT
    return -ahrs.getYaw();
    // ahrs.getRotation2d();
  }

  public double getPitch() {
    return ahrs.getPitch();
  }

  public double getRoll() {
    return ahrs.getRoll();
  }

  public boolean reachedDist(double meters) {
    return (swerveModules[0].reachedDist(meters)) &&
        (swerveModules[1].reachedDist(meters)) &&
        (swerveModules[2].reachedDist(meters)) &&
        (swerveModules[3].reachedDist(meters));
  }

  public void resetDriveEnc() {
    for (SwerveModule s : swerveModules)
      s.resetDriveEnc();
  }

  public Rotation2d getRotation2d() {
    return ahrs.getRotation2d();
    /*
     * the thing is .getYaw is -180 to 180 so it not being 0 to 360
     * may cause the internal conversion that Rotation2d does to be wrong
     */
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber( "Yaw", getYaw());
    for (Integer i = 0; i < 4; ++i) {

      SmartDashboard.putNumber("Swerve" + i.toString() + "angle",
      swerveModules[i].getAngle());
      // SmartDashboard.putNumber("Swerve" + i.toString(),
      // swerveModules[i].getSpeed());
    }

  }

  public void brake() {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].brake();
    }
  }

  public void coast() {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].coast();
    }
  }

  public void stop() {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setSpeed(0.0);
    }
  }
}