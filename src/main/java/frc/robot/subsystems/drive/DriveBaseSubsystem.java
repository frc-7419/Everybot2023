package frc.robot.subsystems.drive;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;


public class DriveBaseSubsystem extends SubsystemBase {

  private final SwerveModule frontLeft = new SwerveModule(
      SwerveConstants.kFrontLeftDriveMotorPort,
      SwerveConstants.kFrontLeftTurningMotorPort,
      SwerveConstants.kFrontLeftDriveEncoderReversed,
      SwerveConstants.kFrontLeftTurningEncoderReversed,
      SwerveConstants.kFrontLeftDriveAbsoluteEncoderPort,
      SwerveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
      SwerveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
      "Front Left");

    private final SwerveModule frontRight = new SwerveModule(
      SwerveConstants.kFrontRightDriveMotorPort,
      SwerveConstants.kFrontRightTurningMotorPort,
      SwerveConstants.kFrontRightDriveEncoderReversed,
      SwerveConstants.kFrontRightTurningEncoderReversed,
      SwerveConstants.kFrontRightDriveAbsoluteEncoderPort,
      SwerveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
      SwerveConstants.kFrontRightDriveAbsoluteEncoderReversed,
      "Front Right");

    private final SwerveModule backLeft = new SwerveModule(
      SwerveConstants.kBackLeftDriveMotorPort,
      SwerveConstants.kBackLeftTurningMotorPort,
      SwerveConstants.kBackLeftDriveEncoderReversed,
      SwerveConstants.kBackLeftTurningEncoderReversed,
      SwerveConstants.kBackLeftDriveAbsoluteEncoderPort,
      SwerveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
      SwerveConstants.kBackLeftDriveAbsoluteEncoderReversed,
      "Back Left");

    private final SwerveModule backRight = new SwerveModule(
      SwerveConstants.kBackRightDriveMotorPort,
      SwerveConstants.kBackRightTurningMotorPort,
      SwerveConstants.kBackRightDriveEncoderReversed,
      SwerveConstants.kBackRightTurningEncoderReversed,
      SwerveConstants.kBackRightDriveAbsoluteEncoderPort,
      SwerveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
      SwerveConstants.kBackRightDriveAbsoluteEncoderReversed,
      "Back Right"); 

  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  public SwerveModulePosition[] getModulePositions(){

    return( new SwerveModulePosition[]{
      frontLeft.getPosition(), 
      frontRight.getPosition(), 
      backLeft.getPosition(),
      backRight.getPosition()});

  }
  private SwerveDriveOdometry odometer;
  public DriveBaseSubsystem() {
    resetAllEncoders();

    new Thread(() -> {
  try {
      Thread.sleep(1000);
      gyro.calibrate();
      zeroHeading();
  } catch (Exception e) {
  }
    }).start();

    odometer = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics, 
    getOdometryAngle(), getModulePositions());
    // new Rotation2d(gyro.getYaw() * -1 / 180 * Math.PI), getModulePositions()

  }

  public void zeroHeading() {
    gyro.reset();
  }
  public void resetYaw(){
    gyro.zeroYaw();
  }

  public void calibrateGyro(){
    gyro.calibrate();
  }
  public double getHeading(){
    return gyro.getAngle();
  }
  public Pose2d getOdometryMeters(){
    return(odometer.getPoseMeters());
  }
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
}

  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose){
   odometer.resetPosition(getOdometryAngle(), getModulePositions(), pose);
  }

  public void resetOdometry(Pose2d pose, Rotation2d rot){
    odometer.resetPosition(rot, getModulePositions(), pose);
  }
  public Rotation2d getOdometryAngle(){
    /* 
    double angle = -gyro.getYaw() + 180;
    if(angle > 180){
      angle -= 360;
    }else if(angle < -180){
      angle += 360;
    }
    return Rotation2d.fromDegrees(angle);
    */
    return(Rotation2d.fromDegrees(gyro.getYaw()));
  }

  public double getRobotDegrees(){
    double rawValue = gyro.getAngle() % 360.0;
    if(rawValue < 0.0){
      return(rawValue + 360.0);
    }else{
      return(rawValue);
    }
  }

  public void resetAllEncoders(){
      frontLeft.resetEncoders();
      frontRight.resetEncoders();
      backLeft.resetEncoders();
      backRight.resetEncoders();
  }

  public double getRumble(){
    return gyro.getRawAccelX();
  }

  public double getRollChange(){
    return(gyro.getRawGyroY());
  }

  public double getRoll(){
    return(gyro.getRoll());
  }

  public double getRobotForceNewtons(){
    return(57.0 * 9.8 * gyro.getRawAccelX());
  }

  @Override
  public void periodic(){

   odometer.update(getOdometryAngle(), getModulePositions());

   // smartdashboard - thanks to sepandar
  //  SmartDashboard.putNumber("Heading", getHeading());
  //  SmartDashboard.putString("Field Location", getPose().getTranslation().toString());
  //  SmartDashboard.putNumber("ROBOT DEGREES NAVX", getRobotDegrees());
  //  SmartDashboard.putString("ODOMETRY", odometer.getPoseMeters().toString());
  //  SmartDashboard.putString("Raw R2d ROBOT DEG", getOdometryAngle().toString());
    
  // SmartDashboard.putBoolean("Gyro Calibrating", gyro.isCalibrating());
  // SmartDashboard.putBoolean("Magnetic Issues", gyro.isMagneticDisturbance());
  // SmartDashboard.putBoolean("Magnetic Calibartion", gyro.isMagnetometerCalibrated());

  // SmartDashboard.putNumber("Robot Acceleration X", gyro.getRawAccelX());
  // SmartDashboard.putNumber("Robot Acceleration Y", gyro.getRawAccelY());
  // SmartDashboard.putNumber("Robot Force X Newtons", 57.0 * 9.8 * gyro.getRawAccelX());
  // SmartDashboard.putNumber("Robot Force X Pounds", (57.0 * 9.8 * gyro.getRawAccelX()) / 4.45);

  // SmartDashboard.putNumber("RAW ROLL", getRoll());
  // SmartDashboard.putNumber("RAW Y", getRollChange());


  frontLeft.update();
  frontRight.update();
  backLeft.update();
  backRight.update();

  }
}