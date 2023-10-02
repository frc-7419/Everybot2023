// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;

public class DriveBaseSubsystem extends SubsystemBase {
  /** Creates a new DriveBaseSubsystem2. */
  private SwerveModule[] swerveModules;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;
  private GyroSubsystem gyroSubsystem;
  private SwerveModulePosition[] positions;
  

  public DriveBaseSubsystem(GyroSubsystem gyroSubsystem) {
    //remember when setting up, swerve0-3 has to be in this orientation: m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation respectively 
    swerveModules = new SwerveModule[] {
      new SwerveModule(SwerveConstants.swerve0.turnMotorID, SwerveConstants.swerve0.speedMotorID, SwerveConstants.swerve0.turnEncoderID),
      new SwerveModule(SwerveConstants.swerve1.turnMotorID, SwerveConstants.swerve1.speedMotorID, SwerveConstants.swerve1.turnEncoderID),
      new SwerveModule(SwerveConstants.swerve2.turnMotorID, SwerveConstants.swerve2.speedMotorID, SwerveConstants.swerve2.turnEncoderID),
      new SwerveModule(SwerveConstants.swerve3.turnMotorID, SwerveConstants.swerve3.speedMotorID, SwerveConstants.swerve3.turnEncoderID),
    };
    positions[0] = getSwerveModule(0).getPosition();
    positions[1] = getSwerveModule(1).getPosition();
    positions[2] = getSwerveModule(2).getPosition();
    positions[3] = getSwerveModule(3).getPosition();

    //TODO: we need the gyro and the module pos for odometry
    this.gyroSubsystem = gyroSubsystem;
    m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(gyroSubsystem.getAngle()), positions, new Pose2d(0, 0, new Rotation2d(0)));
    m_kinematics = new SwerveDriveKinematics(SwerveConstants.swerve0.location, SwerveConstants.swerve1.location, SwerveConstants.swerve2.location, SwerveConstants.swerve3.location); 
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
  
  //call odometry update in this periodic
  @Override
  public void periodic() {
    positions[0] = getSwerveModule(0).getPosition();
    positions[1] = getSwerveModule(1).getPosition();
    positions[2] = getSwerveModule(2).getPosition();
    positions[3] = getSwerveModule(3).getPosition();
    m_odometry.update(new Rotation2d(gyroSubsystem.getAngle()), positions);
  }
}
