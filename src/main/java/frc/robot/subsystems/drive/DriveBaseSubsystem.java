// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.SwerveModule;

public class DriveBaseSubsystem extends SubsystemBase {
  /** Creates a new DriveBaseSubsystem2. */
  private SwerveModule[] swerveModules;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;
  private SwerveModulePosition[] positions;
  private AHRS ahrs;
  private SwerveModule coaster;

  public DriveBaseSubsystem() {
    //remember when setting up, swerve0-3 has to be in this orientation: m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation respectively 
    swerveModules = new SwerveModule[] {
      new SwerveModule(SwerveConstants.frontLeft.turnMotorID, SwerveConstants.frontLeft.speedMotorID, SwerveConstants.frontLeft.turnEncoderID, SwerveConstants.frontLeft.absolutePositionAtRobotZero, 94.219,0),
      new SwerveModule(SwerveConstants.frontRight.turnMotorID, SwerveConstants.frontRight.speedMotorID, SwerveConstants.frontRight.turnEncoderID, SwerveConstants.frontRight.absolutePositionAtRobotZero, 28.125,1),
      new SwerveModule(SwerveConstants.backRight.turnMotorID, SwerveConstants.backRight.speedMotorID, SwerveConstants.backRight.turnEncoderID, SwerveConstants.backRight.absolutePositionAtRobotZero, 266.572,2),
      new SwerveModule(SwerveConstants.backLeft.turnMotorID, SwerveConstants.backLeft.speedMotorID, SwerveConstants.backLeft.turnEncoderID, SwerveConstants.backLeft.absolutePositionAtRobotZero, 267.803,3),
    };
    this.coaster = coaster;
    ahrs = new AHRS(SerialPort.Port.kMXP);
    ahrs.zeroYaw(); //field centric, we need yaw to be zero

    m_kinematics = new SwerveDriveKinematics(SwerveConstants.frontLeft.location, SwerveConstants.frontRight.location, SwerveConstants.backRight.location, SwerveConstants.backLeft.location); 
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        this // Reference to this subsystem to set requirements
    );

    var controller = new HolonomicDriveController(
      new PIDController(1, 0, 0), new PIDController(1, 0, 0),
      new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(6.28, 3.14)));

  }

  public void createPathFinder() {
    // Since we are using a holonomic drivetrain, the rotation component of this pose
    // represents the goal holonomic rotation
    Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        3.0, 4.0, 
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetPose(Pose2d pose) {

  };
  public ChassisSpeeds getRobotRelativeSpeeds() {return getSwerveDriveKinematics().toChassisSpeeds(getSwerveModuleStates());}
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // this is definitely done btw definitely dont work on this later and definitely never come back to this file again fsfr ong
  };
  
  public SwerveModule getSwerveModule(int index) {
    return swerveModules[index];
  }
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return m_kinematics;
  }

  public SwerveModule[] getSwerveModules() {
    return swerveModules;
  }
  public SwerveModuleState[] getSwerveModuleStates() {
    return m_kinematics.toSwerveModuleStates(getRobotRelativeSpeeds());
  }

  public double getYaw() { //CW IS POSITIVE BY DEFAULT
    return -ahrs.getYaw();
    // ahrs.getRotation2d();
  }

  public double getPitch() {
    return ahrs.getPitch();
  }

  public double getRoll() {
    return ahrs.getRoll();
  }

  public Rotation2d getRotation2d() {
    return ahrs.getRotation2d();
    /*the thing is .getYaw is -180 to 180 so it not being 0 to 360 
    may cause the internal conversion that Rotation2d does to be wrong 
    */
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw", getYaw());
    for (Integer i=0; i<4; ++i) {
      SmartDashboard.putNumber("Swerve" + i.toString() + "angle", swerveModules[i].getAngle());
      // SmartDashboard.putNumber("Swerve" + i.toString(), swerveModules[i].getSpeed());
    }
    
  }
  

  public void brake() {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setSpeed(0.0);
    }
  }

  public void coast() {
    coaster.SwerveCoast();
  }
}
