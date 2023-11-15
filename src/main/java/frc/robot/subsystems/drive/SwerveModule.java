package frc.robot.subsystems.drive;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class SwerveModule extends SubsystemBase {
 
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final PIDController turningPidController;
  private SparkMaxPIDController builtinTurningPidController;

  private final DutyCycleEncoder absoluteEncoder;

  //private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  private String moduleName;

  // Class constructor where we assign default values for variable
   public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoLuteEncoderReversed, String name) {

    absoluteEncoderOffsetRad = absoluteEncoderOffset;
    //absoluteEncoderReversed = absoLuteEncoderReversed;

    moduleName = name;

    // this is our absolute encoder 
    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);

    absoluteEncoder.setDutyCycleRange(1.0/4096.0, 4095.0/4096.0);

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPM2MeterPerSec);
    //RADIANS
    turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderRPM2RadPerSec);

   //roborio pid

    turningPidController = new PIDController(SwerveModuleConstants.kPTurning, 0, 0);

    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    // end of roborio pid

    builtinTurningPidController = turningMotor.getPIDController();
    builtinTurningPidController.setP(SwerveModuleConstants.kPTurning);
    builtinTurningPidController.setI(SwerveModuleConstants.kITurning);
    builtinTurningPidController.setD(SwerveModuleConstants.kDTurning);
    builtinTurningPidController.setIZone(0.0);
    builtinTurningPidController.setFF(0.0);
    builtinTurningPidController.setOutputRange(-1, 1);
    turningMotor.burnFlash();

    driveMotor.setIdleMode(IdleMode.kBrake);
    turningMotor.setIdleMode(IdleMode.kBrake);

    driveMotor.setSmartCurrentLimit(40);
    turningMotor.setSmartCurrentLimit(20);
    resetEncoders();



  }

  public void update(){

    SmartDashboard.putNumber(moduleName + "Absolute-Position", absoluteEncoder.getAbsolutePosition());
   // SmartDashboard.putNumber(moduleName + " Turning Position", getTurningPosition());
  }

  // Helpful get methods
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
      return turningEncoder.getPosition();
      //return getAbsoluteEncoderRad();
    }

  public double getDriveVelocity() {
      return driveEncoder.getVelocity();
    }

  public double getTurningVelocity() {
      return turningEncoder.getVelocity();
    }
    
  public SwerveModulePosition getPosition(){
    return( new SwerveModulePosition(
      getDrivePosition(), new Rotation2d(getTurningPosition())));
  }

  public double getAbsoluteEncoderRad(){
    double angle;
    angle = 1 - absoluteEncoder.getAbsolutePosition();

    // Convert into radians
    angle *= 2.0 * Math.PI;
    //angle -= (SmartDashboard.getNumber(moduleName + " ABE Manual", 0) / 180.0) * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    //angle -= absoluteEncoderOffsetRad;

    /*
    if(angle < 0){
      angle = 2.0 * Math.PI + angle ;
    } 
    */

    //angle = Math.abs(angle);

    // Make negative if set
    //angle *= ( absoluteEncoderReversed ? -1.0 : 1.0);
    

    return angle;
    
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state){

    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }

   
    state = SwerveModuleState.optimize(state, getState().angle);

 
    driveMotor.set(state.speedMetersPerSecond / SwerveConstants.kPhysicalMaxSpeedMetersPerSecond);

    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    //turningMotor.set(builtinTurningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    

  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public double[] getMotorsCurrent(){
    return(new double[]{driveMotor.getOutputCurrent(),turningMotor.getOutputCurrent()});
  }

  public double[] getMotorsTemp(){
    return(new double[]{driveMotor.getMotorTemperature(),turningMotor.getMotorTemperature()});
  }

  public void setSmartCurrentLimiter(int driveLimit, int turningLimit){
    driveMotor.setSmartCurrentLimit(driveLimit);
    turningMotor.setSmartCurrentLimit(driveLimit);
  }



}