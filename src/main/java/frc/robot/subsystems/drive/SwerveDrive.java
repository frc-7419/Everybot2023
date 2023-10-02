// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;


//The Reason Why Both SwerveDrive Exists and FieldCentricSwerveDrive Exists, is in case we ever decided robot-centric swerve is better.
public class SwerveDrive extends SubsystemBase {
  private DriveBaseSubsystem driveBaseSubsystem;
  private GyroSubsystem gyroSubsystem;

  /**
   * SwerveDrive Command For Tele-Op Period
   * @param driveBaseSubsystem
   * @param gyroSubsystem
   */
  public SwerveDrive(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.gyroSubsystem = gyroSubsystem;
  }

  /**
   * Returns chassis speeds from field-centric joystick controls
   * @param joystick
   * @return
   */
  public ChassisSpeeds getChassisSpeedsFromJoystick(XboxController joystick) {

    //Make sure there is no joystick drift, YOU CAN REMOVE Deadband if it's not necessary
    double vx = MathUtil.applyDeadband(joystick.getLeftX(), 0.02)*SwerveConstants.maxTranslationalSpeed;
    double vy = MathUtil.applyDeadband(joystick.getLeftY(), 0.02)*SwerveConstants.maxTranslationalSpeed * -1;
    double rx = MathUtil.applyDeadband(joystick.getRightX(), 0.02)*SwerveConstants.maxRotationalSpeed;

    //WPILIB does the Field-Relative Conversions for you, easy peasy
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rx, gyroSubsystem.getRotation2d());
    return speeds;
  }

  /**
   * Converts chassis speeds to individual module speeds
   * @param chassisSpeeds
   * @return 
   */
  public SwerveModuleState[] ChassisSpeedstoModuleSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates = driveBaseSubsystem.getSwerveDriveKinematics().toSwerveModuleStates(chassisSpeeds);
    return moduleStates;
  }
  /**
   * What actually sets the individual swerve moduel states!
   * @param moduleStates
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    for (int i=0; i<4; ++i) {
      driveBaseSubsystem.getSwerveModule(i).setSwerveModuleState(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle);
    }
  }

  /**
   * Sets the module states form the chassis speed
   * @param chassisSpeeds
   */
  public void setModuleStatesFromChassisSpeed(ChassisSpeeds chassisSpeeds) {
    setModuleStates(ChassisSpeedstoModuleSpeeds(chassisSpeeds));
  }

  /**
   * this is what makes the robot begin moving, the entry point for swerve centric drive!
   * @param joystick
   */
  public void setModuleStatesFromJoystick(XboxController joystick) {
    setModuleStatesFromChassisSpeed(getChassisSpeedsFromJoystick(joystick));
  }

  /**
   * This should rotate the robot immediately to alliance wall (with no translation speed)
   */
  public void alignWithAllianceWall() {
    setModuleStatesFromChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, Constants.SwerveConstants.maxRotationalSpeed, Rotation2d.fromDegrees(0)));
  }

  /**
   * This should rotate the robot immediately to opposing wall (with no translation speed)
   */
  public void alignWithOpposingWall() {
    setModuleStatesFromChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, Constants.SwerveConstants.maxRotationalSpeed, Rotation2d.fromDegrees(180)));
  }

  @Override
  public void periodic() {
  }
}
