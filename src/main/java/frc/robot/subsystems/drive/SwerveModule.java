// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;


// The SwerveModule file contains the the logic of the entire Swerve Drive functionality of EveryBot
// Inputs: Motor IDs(CanSparks for now, but change when new motors arrive)
// Outputs: The Swerve Drive functionality for EveryBot
public class SwerveModule {
    private CANSparkMax turnMotor;
    private CANSparkMax speedMotor;
    private CANCoder turnEncoder;
    private RelativeEncoder driveEncoder;
    private PIDController angleController;
    private double cancoderOffset;

    /**
     * Contains the main SwerveModule logic of the bot
     * @param rID is a CAN ID parameter(int)
     * @param sID is a CAN ID parameter(int)
     * @param eID is a CAN ID parameter(int)
     * @param absolutePositionAtRobotZero is absolute pos at zero in deg (double)
     */
    public SwerveModule(int rID, int sID, int eID, double absolutePositionAtRobotZero) {
        turnMotor = new CANSparkMax(rID, MotorType.kBrushless); //assuming two NEOs
        speedMotor = new CANSparkMax(sID, MotorType.kBrushless);
        turnEncoder = new CANCoder(eID);
        cancoderOffset = -absolutePositionAtRobotZero;
        driveEncoder = speedMotor.getEncoder();
  
        config();

        angleController = new PIDController(SwerveConstants.anglekP, 0, 0); //never changes after initialization anyways
        angleController.setTolerance(1); //degrees for now...
        angleController.enableContinuousInput(0, 360); //[-180,180] instead of [0,360) seems to be how most vendor classes implement angle return output in degrees
    }

    public void config() {
        turnEncoder.configFactoryDefault();
        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360); //SushiSquad and Fusion prefer 0 to 360 but whatever
        turnEncoder.configMagnetOffset(cancoderOffset);
    }

    /**
    * Without worrying about factors such as  field-centric drive, chassis conversion, etc, this is the most fundamental method of controlling each swerve module
    * @param speed in m/s (to match with wpilib's format) which is later converted
    * @param rotation2D angle in wpilib's Rotation2D object format
    */
    public void setSwerveModuleState(double speed, Rotation2d rotation2D) {
      setSpeed(speed);
      setAnglePID(rotation2D); 
    }
    
    /**
     * This function sets the speed of the motors
     * @param speed is in the format meters per second(m/s) type: double
     */
    public void setSpeed(double speed) {
        //set refers to percentage motor speed output. Internally it controls voltage (which is surprisingly closely proportional to rpm) and uses a type of setpoint command
        speedMotor.set(speed/ Constants.SwerveConstants.maxTranslationalSpeed);
    }

    /**
    * Rotates the bot to the specified angle with precision
    * @param rotation2D is of type Rotation2d. This is the angle that you want to turn the robot
    */
    public void setAnglePID(Rotation2d rotation2D) {    
        //the units for angle is in degrees now
        double angle = MathUtil.inputModulus(rotation2D.getDegrees(), 0, 360);
        //We should clamp the PID output to between -1 and 1
        turnMotor.set(MathUtil.clamp(angleController.calculate(getAngle(), angle) , -1.0 , 1.0) );
    }

    /**
     * @return angular velocity of individual swerve module in degrees per second
     */
    public double getAnglularVelocity() {
        return turnEncoder.getVelocity();
    }

    
    /**
     * Gets the absolute position of the cancoder, it tells you the angle of rotation of CANCoder. 
     * @return the angle of the bot from the original starting angle
     */
    public double getAngle() {
        return turnEncoder.getPosition(); //make sure this is degrees
    }
    
    /**
     * This function returns the rotation of the degrees from the CANCoder
     * @return the degrees from the CANCoder
    //  */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    //needs to be updated
    public double getSpeed() {
        return Units.degreesToRotations(speedMotor.getEncoder().getVelocity() * SwerveConstants.gearRatioCANCoder) * SwerveConstants.wheelDiameter;
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

    /**
     * Potential use for debugging
     * Puts the data on the SmartDashboard
     */
    public void getSwerveModuleState() {
    
    }

  }
  