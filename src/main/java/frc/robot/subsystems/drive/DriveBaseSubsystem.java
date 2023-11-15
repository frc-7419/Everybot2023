// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBaseSubsystem extends SubsystemBase {
  private final SwerveModule[] swerveModules;
  private SwerveDriveOdometry m_odometry;
  private SwerveModulePosition[] positions;
  private final AHRS ahrs;

  public DriveBaseSubsystem() {
    swerveModules = new SwerveModule[] {
        new SwerveModule(SwerveConstants.frontLeft.turnMotorID, SwerveConstants.frontLeft.driveMotorID,
         SwerveConstants.frontLeft.turnEncoderID, SwerveConstants.frontLeft.absolutePositionAtRobotZero, "FrontLeftModule"),
        new SwerveModule(SwerveConstants.frontRight.turnMotorID, SwerveConstants.frontRight.driveMotorID,
         SwerveConstants.frontRight.turnEncoderID, SwerveConstants.frontRight.absolutePositionAtRobotZero, "FrontRightModule"),
        new SwerveModule(SwerveConstants.backRight.turnMotorID, SwerveConstants.backRight.driveMotorID,
         SwerveConstants.backRight.turnEncoderID, SwerveConstants.backRight.absolutePositionAtRobotZero, "BackRightModule"),
        new SwerveModule(SwerveConstants.backLeft.turnMotorID, SwerveConstants.backLeft.driveMotorID,
         SwerveConstants.backLeft.turnEncoderID, SwerveConstants.backLeft.absolutePositionAtRobotZero, "BackLeftModule"),
    };
    ahrs = new AHRS(SerialPort.Port.kMXP);
    ahrs.zeroYaw(); // field centric, we need yaw to be zero
    coast();
  }

  public void zeroYaw() {
    ahrs.zeroYaw();
  }

  public SwerveModule getSwerveModule(int index) {
    return swerveModules[index];
  }

  public SwerveModule[] getSwerveModules() {
    return swerveModules;
  }

  public double getYaw() { // CW IS POSITIVE BY DEFAULT
    return -ahrs.getYaw();
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
    for(SwerveModule s : swerveModules) {
      s.outputDashboard();
    }
  }
  public void brake() {
    for (SwerveModule s : swerveModules) {
      s.brake();
    }
  }
  public void coast() {
    for (SwerveModule s : swerveModules) {
      s.coast();
    }
  }
  public void stop() {
    for (SwerveModule s : swerveModules) {
      s.setSpeed(0.0);
    }
  }
  /**
   * Returns chassis speeds from field-centric joystick controls. This is what determines the translational speed of the robot in proportion to joystick values.
   * @param joystick
   * @return
   */
  public ChassisSpeeds getChassisSpeedsFromJoystick(XboxController joystick) {
    double vx = Math.abs(joystick.getLeftY())>0.05?joystick.getLeftY() *SwerveConstants.maxTranslationalSpeed:0;
    double vy = Math.abs(joystick.getLeftX())>0.05?joystick.getLeftX()*SwerveConstants.maxTranslationalSpeed:0;
    double rx = Math.abs(joystick.getRightX())>0.05?-0.7*joystick.getRightX()*SwerveConstants.maxRotationalSpeed:0;
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rx, getRotation2d());
    return speeds;
  }
    /**
   * Converts chassis speeds to individual module speeds
   * @param chassisSpeeds
   * @return 
   */
  public SwerveModuleState[] ChassisSpeedstoModuleSpeeds(ChassisSpeeds chassisSpeeds) {
    return Constants.SwerveConstants.m_kinematics.toSwerveModuleStates(chassisSpeeds);
  }
}