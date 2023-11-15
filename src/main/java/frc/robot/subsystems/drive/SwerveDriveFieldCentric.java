package frc.robot.subsystems.drive;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveFieldCentric extends CommandBase {

  private final DriveBaseSubsystem driveBaseSubsystem;
  private final double xSpdFunction, ySpdFunction, turningSpdFunction, headingFunction;
  private final boolean fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final boolean flickFunction;
  private double initialHeading;
  private PIDController thetaController;
  
  private SendableChooser<Double> autoChooser = new SendableChooser<>();
 
  private double added;
  private int counter;
  public SwerveDriveFieldCentric(DriveBaseSubsystem driveBaseSubsystem,
  double xSpdFunction, double ySpdFunction, double turningSpdFunction,
  boolean fieldOrientedFunction, double headingFunction, boolean flickFunction){
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.headingFunction = headingFunction;

      this.flickFunction = flickFunction;
      this.initialHeading = headingFunction;
      this.xLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
      thetaController = new PIDController(SwerveConstants.kPThetaController, SwerveConstants.kIThetaController, SwerveConstants.kDThetaController);

      added = 0;
      counter = 0;

      // Tell command that it needs subsstytemmem
      addRequirements(driveBaseSubsystem);

    }

    @Override
    public void initialize() {
      initialHeading = headingFunction;
      added = 0;
    }

    @Override
    public void execute(){

      double xSpeed = xSpdFunction;
      double ySpeed = ySpdFunction;
      double turningSpeed = turningSpdFunction * 8;

      xSpeed = Math.abs(xSpeed) > SwerveConstants.kDeadband ? xSpeed : 0.0;
      ySpeed = Math.abs(ySpeed) > SwerveConstants.kDeadband ? ySpeed : 0.0;
      turningSpeed = Math.abs(turningSpeed) > SwerveConstants.kDeadband ? turningSpeed : 0.0;

      xSpeed = xLimiter.calculate(xSpeed) * SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond;
      ySpeed = yLimiter.calculate(ySpeed) * SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    

      initialHeading += turningSpeed;


      double newHeading = headingFunction;

  
      if(flickFunction){
        counter += 1;
      if(counter > 6){
        added += 90;
        counter = 0;
      }
    }else{counter = 0;}

    turningSpeed = thetaController.calculate(newHeading - added, initialHeading) * 100;
    //turningSpeed = (headingFunction.get() - initialHeading) * turningPValue;
    

    turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;
    turningSpeed *= -1;


    if (turningSpeed > SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond){
      turningSpeed = SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }
    else if (turningSpeed < -SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond){
      turningSpeed = -SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }
    
    SmartDashboard.putNumber("Turning Speed", turningSpeed);
    SmartDashboard.putNumber("Inital Heading", initialHeading);
    SmartDashboard.putNumber("NavX Heading", headingFunction);


    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, Rotation2d.fromDegrees(driveBaseSubsystem.getRobotDegrees()));
    //chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turningSpeed);
    SwerveModuleState[] moduleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    driveBaseSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted){driveBaseSubsystem.stopModules();}

  @Override
  public boolean isFinished(){return false;}

}