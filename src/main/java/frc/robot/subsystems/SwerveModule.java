// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;


// The SwerveModule file contains the the logic of the entire Swerve Drive functionality of EveryBot
// Inputs: Motor IDs(CanSparks for now, but change when new motors arrive)
// Outputs: The Swerve Drive functionality for EveryBot
public class SwerveModule {
    private CANSparkMax turnMotor;
    private CANSparkMax driveMotor;
    private CANCoder turnEncoder;
    private RelativeEncoder driveEncoder;
    private PIDController angleController;
    

    /**
     * Contains the main SwerveModule logic of the bot
     * @param driveMotorID is a CAN ID parameter(int)
     * @param turnMotorID is a CAN ID parameter(int)
     * @param turnEncoderID is a CAN ID parameter(int)
     */
    public SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderID) {
        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        this.turnEncoder = new CANCoder(turnEncoderID);
        config();
        angleController = new PIDController(SwerveConstants.anglekP, 0, 0); //never changes after initialization anyways
        angleController.setTolerance(1); //degrees for now...
        angleController.enableContinuousInput(-180, 180); //[-180,180] instead of [0,360) seems to be how most vendor classes implement angle return output in degrees
    }

    // TODO: Change the functions and declarations after new motors arrive
    public void config() {
        turnEncoder.configFactoryDefault();
        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); //SushiSquad and Fusion prefer 0 to 360 but whatever
    }

    /**
    * Without worrying about factors such as  field-centric drive, chassis conversion, etc, this is the most fundamental method of controlling each swerve module
    * @param speed in m/s (to match with wpilib's format) which is later converted
    * @param rotation2D angle in wpilib's Rotation2D object format
    */
    public void setSwerveModuleState(SwerveModuleState state) {
      setSpeed(state.speedMetersPerSecond);
      setAnglePID(state.angle); 
    }
    
    /**
     * This function sets the speed of the motors
     * @param speed is in the format meters per second(m/s) type: double
     */
    public void setSpeed(double speed) {
        //set refers to percentage motor speed output. Internally it controls voltage (which is surprisingly closely proportional to rpm) and uses a type of setpoint command
        driveMotor.set(speed/SwerveConstants.maxSpeed); 
    }

    /**
    * turns the bot to the specified angle with precision
    * @param rotation2D is of type Rotation2d. This is the angle that you want to turn the robot
    */
    public void setAnglePID(Rotation2d rotation2D) {    
        //the units for angle is in degrees now
        double angle = MathUtil.inputModulus(rotation2D.getDegrees(), -180, 180);
        //We should clamp the PID output to between -1 and 1
        driveMotor.set(MathUtil.clamp( angleController.calculate(getAngle(), angle) , -1.0 , 1.0) );
    }

    /**
     * @return angular velocity of individual swerve module in degrees per second
     */
    public double getAnglularVelocity() {
        return turnEncoder.getVelocity();
    }

    
    /**
     * Gets the absolute position of the canCoder, although it will not tell you wheel rotation, instead speed. 
     * @return the angle of the bot from the original starting angle
     */
    public double getAngle() { //from what I've seen the CANCODER does not tell you anything about wheel rotation but rather speed
        return turnEncoder.getPosition();
    }
    
    
    /**
     * This function returns the rotation of the degrees from the CANCoder
     * @return the degrees from the CANCoder
    //  */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    //from what I've seen the CANCODER does not tell you anything about wheel rotation but rather speed
    public double getSpeed() {
        return Units.degreesToRotations(driveEncoder.getVelocity() * SwerveConstants.gearRatioCANCoder) * SwerveConstants.wheelDiameter;
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * This function returns the displacement of the bot from the origin
     * @return the displacement of the bot from the origin
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getRotation2d());
    }

    public void brake() {
        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kBrake);
    }

    public void coast() {
        driveMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setPower(double power){
        driveMotor.set(power);
        turnMotor.set(power);
    }


    /**
     * Potential use for debugging
     * Puts the data on the SmartDashboard
     */
    public void getSwerveModuleState() {

    }

  }
  