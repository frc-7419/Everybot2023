// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;


// The SwerveModule file contains the the logic of the entire Swerve Drive functionality of EveryBot
// Inputs: Motor IDs(CanSparks for now, but change when new motors arrive)
// Outputs: The Swerve Drive functionality for EveryBot
public class SwerveModule {
    private final CANSparkMax turnMotor;
    private final CANSparkMax driveMotor;
    private final CANCoder turnEncoder;
    private final RelativeEncoder driveEncoder;
    private final PIDController angleController;
    private final double turnEncoderOffset;
    private final String module;

    /**
     * Contains the main SwerveModule logic of the bot
     * @param turnMotorID is a CAN ID parameter(int)
     * @param driveMotorID is a CAN ID parameter(int)
     * @param turnEncoderID is a CAN ID parameter(int)
     * @param turnEncoderOffset is absolute pos at zero in deg (double)
     * @param module for naming modules during comprehensive shuffleboard outputs
     */
    public SwerveModule(int turnMotorID, int driveMotorID, int turnEncoderID, double turnEncoderOffset, String module) {
        this.module = module;
        this.turnEncoderOffset = turnEncoderOffset;
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnEncoder = new CANCoder(turnEncoderID);
        driveEncoder = driveMotor.getEncoder();
        angleController = new PIDController(0.003, 0, 0);
        turnMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setIdleMode(IdleMode.kCoast);
        angleController.setTolerance(0.5);
        angleController.enableContinuousInput(0, 360);
        turnEncoder.configFactoryDefault();
        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnEncoder.configMagnetOffset(turnEncoderOffset);
        turnEncoder.configSensorDirection(false);
        resetToAbsolute();
        driveEncoder.setPositionConversionFactor(1/22);
    }
    private void resetToAbsolute() {
        turnEncoder.setPosition(turnEncoder.getPosition() + turnEncoderOffset);
      }

    public void coast() {
        turnMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setIdleMode(IdleMode.kCoast);
    }
    
    public void brake() {
        turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setIdleMode(IdleMode.kBrake);
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    public void resetDriveEnc() {
        driveEncoder.setPosition(0);
    }
    public boolean reachedDist(double meters) {
        return Math.abs(driveEncoder.getPosition()) > meters;
    }
    // TODO: broken set factor
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }
    // TODO: broken set factor
    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }
    // TODO: fix getDriveVelocity()
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(turnEncoder.getPosition()));
    }
    public void setSwerveModuleState(SwerveModuleState state) {
        setSpeed(state.speedMetersPerSecond);
        setAnglePID(state.angle);
    }
    public void setSwerveModuleState(SwerveModuleState state, XboxController joystick) {
        setSpeed(state.speedMetersPerSecond, joystick);
        setAnglePID(state.angle);
    }
    public void testTurn(){
        turnMotor.set(0.1);
    }
    
    /**
     * This function sets the speed of the motors
     * @param speed is in the format meters per second(m/s) type: double
     */
    public void setSpeed(double speed) {
        double motorInput = speed/Constants.SwerveConstants.maxTranslationalSpeed;
        driveMotor.set(motorInput);
    }
    /**
     * This function sets the speed of the motors
     * @param speed is in the format meters per second(m/s) type: double
     */
    public void setSpeed(double speed, XboxController joystick) {
        double motorInput = speed/Constants.SwerveConstants.maxTranslationalSpeed;
        motorInput = joystick.getLeftBumper()?motorInput*0.2:motorInput;
        driveMotor.set(motorInput);
    }
    public void stop() {
        resetToAbsolute();
        driveMotor.set(0);
        turnMotor.set(0);
      }
    

    /**
    * Rotates the bot to the specified angle with precision
    * @param rotation2D is of type Rotation2d. This is the angle that you want to turn the robot
    */
    public void setAnglePID(Rotation2d rotation2D) {   
        double angleSetpoint = rotation2D.getDegrees(); // 0 to 360!
        double PIDVAL = angleController.calculate(getAngle(), angleSetpoint);
        double PIDVALCLAMP = MathUtil.clamp(PIDVAL , -0.3 , 0.3);
        turnMotor.set(-PIDVALCLAMP);
    }
    
    /**
     * Gets the absolute position of the cancoder, it tells you the angle of rotation of CANCoder. 
     * @return the angle of the bot from the original starting angle
     */
    public double getAngle() {
        return (turnEncoder.getAbsolutePosition()); //make sure this is degrees
    }
    
  }