// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Gyro.GyroSubsystem;

public class SwerveDriveFieldCentric extends CommandBase {
  /** Creates a new RunSwerveWithJoystick. */
  private XboxController joystick;
  private DrivebaseSubsystem drivebaseSubsystem;
  private GyroSubsystem gyroSubsystem;

  private SwerveDriveOdometry m_odometry;
  private Pose2d m_pose;

 // Sets the joystick, driveBaseSubsystem and gyroSubsystem.
  public SwerveDriveFieldCentric(XboxController joystick, DrivebaseSubsystem drivebaseSubsystem, GyroSubsystem gyroSubsystem) {
    this.joystick = joystick;
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.gyroSubsystem = gyroSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebaseSubsystem, gyroSubsystem);
  }
  
  /*
  * How will Swerve Work?
  * Joysticks need to output a x/y speed and a rotation theta speed
  * Always REMEMBER this is FIELD-ORIENTED DRIVE
  */

  // Called when the command is initially scheduled.

  // Gets the swerve position when the command is initially scheduled
  @Override
  public void initialize() {
    m_odometry = new SwerveDriveOdometry(
    drivebaseSubsystem.getSwerveDriveKinematics(), gyroSubsystem.getRotation2d(),
    new SwerveModulePosition[] {
      drivebaseSubsystem.getSwerveModule(0).getSwerveModulePosition(),
      drivebaseSubsystem.getSwerveModule(1).getSwerveModulePosition(),
      drivebaseSubsystem.getSwerveModule(2).getSwerveModulePosition(),
      drivebaseSubsystem.getSwerveModule(3).getSwerveModulePosition()
    }, new Pose2d(0, 0, new Rotation2d(0)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setSwerveModuleStates(ChassisSpeedstoModuleSpeeds(getChassisSpeedsFromJoystick())); 
    
  }
  // Called to update the swerve position
  public void updatePose() {
    m_pose = m_odometry.update(gyroSubsystem.getRotation2d(),
    new SwerveModulePosition[] {
      drivebaseSubsystem.getSwerveModule(0).getSwerveModulePosition(),
      drivebaseSubsystem.getSwerveModule(1).getSwerveModulePosition(),
      drivebaseSubsystem.getSwerveModule(2).getSwerveModulePosition(),
      drivebaseSubsystem.getSwerveModule(3).getSwerveModulePosition()
    });
  }

  public ChassisSpeeds getChassisSpeedsFromJoystick() {
    //Make sure there is no joystick drift, YOU CAN REMOVE Deadband if it's not necessary
    double vx = MathUtil.applyDeadband(joystick.getLeftX(), 0.02)*SwerveConstants.maxSpeed;
    double vy = MathUtil.applyDeadband(joystick.getLeftY(), 0.02)*SwerveConstants.maxSpeed;
    double rx = MathUtil.applyDeadband(joystick.getRightX(), 0.02)*SwerveConstants.maxSpeed;
    
    //WPILIB does the Field-Relative Conversions for you, easy peasy
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rx, gyroSubsystem.getRotation2d());
    return speeds;
  }
  // Sets the module states to the chassis speeds. 
  public SwerveModuleState[] ChassisSpeedstoModuleSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates = drivebaseSubsystem.getSwerveDriveKinematics().toSwerveModuleStates(chassisSpeeds);
    return moduleStates;
  }

  public void setSwerveModuleStates(SwerveModuleState[] moduleStates) {
    for (int i=0; i<4; ++i) {
      drivebaseSubsystem.getSwerveModule(i).setSwerveModuleState(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}