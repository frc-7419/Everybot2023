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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnEncoder = new CANCoder(turnEncoderID);
        driveEncoder = driveMotor.getEncoder();
        angleController = new PIDController(Constants.SwerveConstants.anglekP, Constants.SwerveConstants.anglekI, Constants.SwerveConstants.anglekD);
        turnMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setIdleMode(IdleMode.kCoast);
        angleController.setTolerance(1);
        angleController.enableContinuousInput(0, 360);
        turnEncoder.configFactoryDefault();
        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnEncoder.configMagnetOffset(turnEncoderOffset);
        turnEncoder.configSensorDirection(false);
        driveEncoder.setPositionConversionFactor(1/22); // TODO: fix this
        resetDriveEnc();
    }

    public void coast() {
        turnMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setIdleMode(IdleMode.kCoast);
    }
    
    public void brake() {
        turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setIdleMode(IdleMode.kBrake);
    }
    // TODO: meter is not a meter
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    public void resetDriveEnc() {
        driveEncoder.setPosition(0);
    }
    // TODO: meter is not a meter
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
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
        driveMotor.set(state.speedMetersPerSecond);
        turnMotor.set(-MathUtil.clamp(angleController.calculate(turnEncoder.getAbsolutePosition(), state.angle.getDegrees()) , -0.3 , 0.3));
    }   
    /**
     * This function sets the speed of the motors
     * @param speed is in the format meters per second(m/s) type: double
     */
    public void setSpeed(double speed) {
        driveMotor.set(speed/Constants.SwerveConstants.maxTranslationalSpeed);
    }
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
    
    public void outputDashboard() {
        SmartDashboard.putNumber(module+" angle", turnEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(module+" driveEncoder", getDrivePosition());
    }
  }