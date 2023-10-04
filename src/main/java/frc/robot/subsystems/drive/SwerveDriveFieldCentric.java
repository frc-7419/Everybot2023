// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;

public class SwerveDriveFieldCentric extends CommandBase {
  private XboxController joystick;
  private DriveBaseSubsystem driveBaseSubsystem;
  private GyroSubsystem gyroSubsystem;

    /*
  * How will Swerve Work?
  * Joysticks need to output a x/y speed and a rotation theta speed
  * Always REMEMBER this is FIELD-ORIENTED DRIVE
  */
  public SwerveDriveFieldCentric(XboxController joystick, DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.gyroSubsystem = gyroSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBaseSubsystem, gyroSubsystem);
  }
  
  /**
   * Returns chassis speeds from field-centric joystick controls. This is what determines the translational speed of the robot in proportion to joystick values.
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
   * Sets the individual swerve module states
   * @param moduleStates
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    for (int i=0; i<4; ++i) {
      driveBaseSubsystem.getSwerveModule(i).setSwerveModuleState(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle);
    }
  }

  //The following two function smake the code less verbose by combining the above functions
  /**
   * Sets the module states directly from the chassis speed
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
    setModuleStatesFromChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
  }

  /**
   * This should rotate the robot immediately to opposing wall (with no translation speed)
   */
  public void alignWithOpposingWall() {
    setModuleStatesFromChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(180)));
  }

  /**
   * Set swerve modules to its zero state. Note that the CANCoders must be zeroed to their correct position first...
   */
  public void zero() {
    setModuleStatesFromChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // zero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setModuleStatesFromJoystick(joystick);

    // //AUTO ALIGN PREVIEW
    // if (joystick.getLeftBumper() ) {
    //   alignWithAllianceWall();
    // }
    // else if (joystick.getRightBumper() ) {
    //   alignWithOpposingWall();
    // }
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