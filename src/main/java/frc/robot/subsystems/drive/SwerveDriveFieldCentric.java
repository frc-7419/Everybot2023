// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveDriveFieldCentric extends CommandBase {
  private XboxController joystick;
  private static DriveBaseSubsystem driveBaseSubsystem;
  public double recordedAngle;

  public SwerveDriveFieldCentric(XboxController joystick, DriveBaseSubsystem driveBaseSubsystem) {
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
    addRequirements(driveBaseSubsystem);
  }
  
  /**
   * Returns chassis speeds from field-centric joystick controls. This is what determines the translational speed of the robot in proportion to joystick values.
   * @param joystick
   * @return
   */
  public ChassisSpeeds getChassisSpeedsFromJoystick(XboxController joystick) {

    //DEADBAND WAS WHY FOWARD/BACKWARD DIDNT WORK
    double maxTranslationalSpeed = SwerveConstants.maxTranslationalSpeed;
    double maxTranslationalSpeedX = SwerveConstants.maxTranslationalSpeedX;

    double vx = -(Math.abs(joystick.getLeftY()) > 0.05 ? joystick.getLeftY() : 0.0) *SwerveConstants.maxTranslationalSpeedX;
    double vy = -(Math.abs(joystick.getLeftX()) > 0.05 ? joystick.getLeftX() : 0.0)*SwerveConstants.maxTranslationalSpeed ;
    double rx = joystick.getRightX()*SwerveConstants.maxRotationalSpeed;

    // if(joystick.getLeftY() <-0.05 ){
    //   rx = 0;
    //   maxTranslationalSpeedX = maxTranslationalSpeedX*-1;
    // } this code dont work

    // SmartDashboard.putNumber("LeftX", joystick.getLeftX());
    // SmartDashboard.putNumber("LeftY", joystick.getLeftY());
    // SmartDashboard.putNumber("RightX", joystick.getRightX());

    // SmartDashboard.putNumber("vx", vx);
    // SmartDashboard.putNumber("vy", vy);
    // SmartDashboard.putNumber("omega", rx);

    //WPILIB does the Field-Relative Conversions for you, easy peas y
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rx, driveBaseSubsystem.getRotation2d());

    // discretizing for second-order kinematics
    Pose2d deltaPose = new Pose2d(vx * Constants.RobotConstants.loopDt, vy * Constants.RobotConstants.loopDt, new Rotation2d(rx * Constants.RobotConstants.loopDt));
    Twist2d twist = new Pose2d().log(deltaPose);
    SmartDashboard.putNumber("Robot omega", speeds.omegaRadiansPerSecond);
    return new ChassisSpeeds(twist.dx / Constants.RobotConstants.loopDt, twist.dy / Constants.RobotConstants.loopDt, twist.dtheta / Constants.RobotConstants.loopDt);


    // SmartDashboard.putNumber("Robot vx", speeds.vxMetersPerSecond);
    // SmartDashboard.putNumber("Robot vy", speeds.vyMetersPerSecond);
    
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
      // SmartDashboard.putNumber("Setpoint Speed of Module" + String.valueOf(i), moduleStates[i].speedMetersPerSecond);
      // SmartDashboard.putNumber("Setpoint Angle of Module" + String.valueOf(i), moduleStates[i].angle.getDegrees()); 
      driveBaseSubsystem.getSwerveModule(i).setSwerveModuleState(moduleStates[i]);
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
   * Set swerve modules to its zero state. Note that the CANCoders must be zeroed to their correct position first...
   */
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    recordedAngle =driveBaseSubsystem.getPitch();
    // zero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setModuleStatesFromJoystick(joystick);
    // if (joystick.getAButton()) {
    //   zero();
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