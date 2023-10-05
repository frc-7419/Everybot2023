// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class DriveBaseSubsystem extends SubsystemBase {
  /** Creates a new DriveBaseSubsystem. */
  private SwerveDriveOdometry m_odometry;
  private SwerveDriveKinematics m_kinematics = Constants.SwerveConstants.swerveDriveKinematics;
  private AHRS gyro;
  private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveConstants.frontLeft.driveMotorID,
                                                          Constants.SwerveConstants.frontLeft.turnMotorID,
                                                          Constants.SwerveConstants.frontLeft.turnEncoderID);
  private final SwerveModule frontRight = new SwerveModule(Constants.SwerveConstants.frontRight.driveMotorID,
                                                          Constants.SwerveConstants.frontRight.turnMotorID,
                                                          Constants.SwerveConstants.frontRight.turnEncoderID);
  private final SwerveModule backLeft = new SwerveModule(Constants.SwerveConstants.backLeft.driveMotorID,
                                                          Constants.SwerveConstants.backLeft.turnMotorID,
                                                          Constants.SwerveConstants.backLeft.turnEncoderID);
  private final SwerveModule backRight = new SwerveModule(Constants.SwerveConstants.backRight.driveMotorID,
                                                          Constants.SwerveConstants.backRight.turnMotorID,
                                                          Constants.SwerveConstants.backRight.turnEncoderID);
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
  }
  public DriveBaseSubsystem() {
    try { //for now just do what 7419 did. pretty sure MXP refers to the expansion port on the roborio
      /*
       * Communicate w/navX-MXP via the MXP SPI Bus (use mini USB to USB A cable)
       * Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or S
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      gyro = new AHRS(SerialPort.Port.kMXP);
      SmartDashboard.putString("subsystem", "init gyro sub");
    } 
    catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
    m_odometry = new SwerveDriveOdometry(m_kinematics, getGryoRotation2d(), getSwerveModulePositions());
  }
  public double getAngle(){
    return gyro.getAngle();
  }
  public double getYaw() {
    return gyro.getYaw();
  }
  public double getPitch() {
    return gyro.getPitch();
  }
  public double getRoll() {
    return gyro.getRoll();
  }
  public void resetGyro() {
    gyro.reset();
  }
  public void resetYaw(){
    gyro.zeroYaw();
  }
  public void calibrateGyro(){
    gyro.calibrate();
  }
  public Rotation2d getGryoRotation2d() {
    return Rotation2d.fromDegrees(gyro.getYaw());
    /*the thing is .getYaw is -180 to 180 so it not being 0 to 360 
    may cause the internal conversion that Rotation2d does to be wrong 
    */
  }
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    frontLeft.setSwerveModuleState(moduleStates[0]);
    frontRight.setSwerveModuleState(moduleStates[1]);
    backLeft.setSwerveModuleState(moduleStates[2]);
    backRight.setSwerveModuleState(moduleStates[3]);
  }
  public ChassisSpeeds getChassisSpeedsFromJoystick(XboxController joystick) {
    //Make sure there is no joystick drift, YOU CAN REMOVE Deadband if it's not necessary
    double vx = MathUtil.applyDeadband(joystick.getLeftX(), 0.02)*SwerveConstants.maxSpeed;
    double vy = MathUtil.applyDeadband(joystick.getLeftY(), 0.02)*SwerveConstants.maxSpeed * -1;
    double rx = MathUtil.applyDeadband(joystick.getRightX(), 0.02)*SwerveConstants.maxSpeed;
    //WPILIB does the Field-Relative Conversions for you, easy peasy
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rx, gyro.getRotation2d());
    return speeds;
  }
  public SwerveModuleState[] ChassisSpeedstoModuleSpeeds(ChassisSpeeds chassisSpeeds) {
    return m_kinematics.toSwerveModuleStates(chassisSpeeds);
  }
  public void setModuleStatesFromChassisSpeed(ChassisSpeeds chassisSpeeds) {
    setModuleStates(ChassisSpeedstoModuleSpeeds(chassisSpeeds));
  }
  public void setModuleStatesFromJoystick(XboxController joystick) {
    setModuleStatesFromChassisSpeed(getChassisSpeedsFromJoystick(joystick));
  }
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, DriveBaseSubsystem driveBaseSubsystem) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                this.resetOdometry(traj.getInitialPose());
            }
        }),
        new PPSwerveControllerCommand(
            traj,
            this::getPose, // Pose supplier
            this.m_kinematics, // SwerveDriveKinematics
            new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
            new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
            this::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
  }


  public void resetOdometry(Pose2d initialPose) {
    m_odometry.resetPosition(getGryoRotation2d(), getSwerveModulePositions(), initialPose);
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber(   "Yaw", gyro.getYaw());
    SmartDashboard.putNumber(   "Pitch", gyro.getPitch());
    SmartDashboard.putNumber(   "Roll", gyro.getRoll());
    m_odometry.update(getGryoRotation2d(), getSwerveModulePositions());
  }
}
