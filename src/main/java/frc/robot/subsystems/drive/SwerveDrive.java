// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;

public class SwerveDrive extends SubsystemBase {
  private DriveBaseSubsystem driveBase;
  private GyroSubsystem gryo;
  /**
   * Makes a new SwerveDrive, this is used to control the modules
   * @param driveBase
   * @param gryo
   */
  public SwerveDrive(DriveBaseSubsystem driveBase, GyroSubsystem gryo) {
    this.driveBase = driveBase;
    //gryo lmao
    this.gryo = gryo;
  }
  /**
   * Gets joystick and returns the chassis speed
   * @param joystick
   * @return
   */
  public ChassisSpeeds getChassisSpeedsFromJoystick(XboxController joystick) {
    //Make sure there is no joystick drift, YOU CAN REMOVE Deadband if it's not necessary
    double vx = MathUtil.applyDeadband(joystick.getLeftX(), 0.02)*SwerveConstants.maxSpeed;
    double vy = MathUtil.applyDeadband(joystick.getLeftY(), 0.02)*SwerveConstants.maxSpeed * -1;
    double rx = MathUtil.applyDeadband(joystick.getRightX(), 0.02)*SwerveConstants.maxSpeed;
    //WPILIB does the Field-Relative Conversions for you, easy peasy
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rx, gryo.getRotation2d());
    return speeds;
  }
  /**
   * Converts chassis speeds to module speeds
   * @param chassisSpeeds
   * @return
   */
  public SwerveModuleState[] ChassisSpeedstoModuleSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates = driveBase.getSwerveDriveKinematics().toSwerveModuleStates(chassisSpeeds);
    return moduleStates;
  }
  /**
   * Sets module states, this is what makes the robot begin moving
   * @param moduleStates
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    for (int i=0; i<4; ++i) {
      driveBase.getSwerveModule(i).setSwerveModuleState(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle);
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
   * sets the module states from the joystick,can be used for teleop
   * @param joystick
   */
  public void setModuleStatesFromJoystick(XboxController joystick) {
    setModuleStatesFromChassisSpeed(getChassisSpeedsFromJoystick(joystick));
  }
  @Override
  public void periodic() {
  }
}
