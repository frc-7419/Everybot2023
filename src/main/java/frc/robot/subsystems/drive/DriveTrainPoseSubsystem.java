// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.Constants;

public class DriveTrainPoseSubsystem extends SubsystemBase {

  private final GyroSubsystem gyroSubsystem;
  private final DriveBaseSubsystem driveBaseSubsystem;
  private Pose2d m_pose;
  private DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveTrainPoseSubsystem. */
  public DriveTrainPoseSubsystem(GyroSubsystem gyroSubsystem, DriveBaseSubsystem driveBaseSubsystem) {
    this.gyroSubsystem = gyroSubsystem;
    this.driveBaseSubsystem = driveBaseSubsystem;
    m_pose = new Pose2d(0, 0, new Rotation2d()); 
    
    //for now, i dont see the use
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.RobotConstants.kTrackWidth);

    m_odometry = new DifferentialDriveOdometry(gyroSubsystem.getRotation2d(), 
    driveBaseSubsystem.getLeftDistance(),
    driveBaseSubsystem.getRightDistance(),
    new Pose2d(0, 0, new Rotation2d()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the pose
    m_pose = m_odometry.update(gyroSubsystem.getRotation2d(), driveBaseSubsystem.getLeftDistance(), driveBaseSubsystem.getRightDistance());
    putPose();
  }

  public void resetPose() {
    m_odometry.resetPosition(gyroSubsystem.getRotation2d(), 
    driveBaseSubsystem.getLeftDistance(), 
    driveBaseSubsystem.getRightDistance(), m_pose);
    // "If at any time, you decide to reset your gyroscope or encoders, the resetPosition method 
    // MUST be called with the new gyro angle and wheel distances."
  }

  public void putPose() { //to SmartDashboard
    SmartDashboard.putNumber("X Position (Pose2D)", m_pose.getX());
    SmartDashboard.putNumber("Y Position (Pose2D)", m_pose.getY());
    SmartDashboard.putNumber("Rotation (Pose2D Degrees)", m_pose.getRotation().getDegrees());
  }
}
