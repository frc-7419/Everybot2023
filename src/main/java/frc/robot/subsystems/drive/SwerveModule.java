// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.drive;

// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoder;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.Constants.SwerveModuleConstants;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.util.Units;

// /** Add your docs here. */
// public class SwerveModule {
//     private CANSparkMax rotateMotor;
//     private CANSparkMax speedMotor;
//     private CANCoder canCoder;
//     private int moduleNumber;
  
//     private PIDController angleController;
  
//     public SwerveModule(int rID, int sID, int eID, int moduleNumber) {

//         this.moduleNumber = moduleNumber;

//         rotateMotor = new CANSparkMax(rID, MotorType.kBrushless); //assuming two NEOs
//         speedMotor = new CANSparkMax(sID, MotorType.kBrushless);
//         canCoder = new CANCoder(eID);
  
//         config();

//         angleController = new PIDController(SwerveConstants.anglekP, 0, 0); //never changes after initialization anyways
//         angleController.setTolerance(1); //degrees for now...
//         angleController.enableContinuousInput(-180, 180); //[-180,180] instead of [0,360) seems to be how most vendor classes implement angle return output in degrees

//     }

    
//     public void config() {

//         canCoder.configFactoryDefault();
//         canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
//         canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); //SushiSquad and Fusion prefer 0 to 360 but whatever

//     }

//     /**
//      * Without worrying about factors such as  field-centric drive, chassis conversion, etc, this is the most fundamental method of controlling each swerve module
//      * @param speed in m/s (to match with wpilib's format) which is later converted
//      * @param rotation2D angle in wpilib's Rotation2D object format
//      */
//     public void setSwerveModuleState(double speed, Rotation2d rotation2D) {

//       setSpeed(speed);
//       setAnglePID(rotation2D); 

//     }
  
//     public void setSpeed(double speed) {

//         //set refers to percentage motor speed output. Internally it controls voltage (which is surprisingly closely proportional to rpm) and uses a type of setpoint command
//         speedMotor.set(speed/SwerveConstants.maxSpeed); 

//     }

//     public void setAnglePID(Rotation2d rotation2D) {    
//         //the units for angle is in degrees now
        
//         double angle = MathUtil.inputModulus(rotation2D.getDegrees(), -180, 180);
//         //We should clamp the PID output to between -1 and 1
//         rotateMotor.set(MathUtil.clamp( angleController.calculate(getAngle(), angle) , -1.0 , 1.0) );
        
//     }

//     /**
//      * @return angular velocity of individual swerve module in degrees per second
//      */
//     public double getAnglularVelocity() {
      
//         return canCoder.getVelocity();

//     }

//     public double getAngle() { 
//       //from what I've seen the CANCODER does not tell you anything about wheel rotation but rather speed
//       return canCoder.getAbsolutePosition();
        
//     }

//     public Rotation2d getRotation2d() {

//         return Rotation2d.fromDegrees(getAngle());
        
//     }

//     //from what I've seen the CANCODER does not tell you anything about wheel rotation but rather speed
//     public double getSpeed() {

//         return Units.degreesToRotations(canCoder.getVelocity() * SwerveConstants.gearRatioCANCoder) * SwerveConstants.wheelDiameter;
        
//     }

//     public double getPosition() {
//        //placeholder for now
//         return 0.0;
        
//     }

//     public SwerveModulePosition getSwerveModulePosition() {

//         return new SwerveModulePosition(getPosition(), getRotation2d());
        
//     }

//     /**

//      * Potential use for debugging
     
//      */
//     public void getSwerveModuleState() {
//         SmartDashboard.putNumber("Swerve Mod " + moduleNumber + " Angle", getAngle());
//         SmartDashboard.putNumber("Swerve Mod " + moduleNumber + " Speed", getSpeed());
//     }

//   }