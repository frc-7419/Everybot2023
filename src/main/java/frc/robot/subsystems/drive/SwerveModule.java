// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

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
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
    private int rID;
    private int sID;
    private int eID;
    private double offset;
    private int module;
    DriveBaseSubsystem driveBaseSubsystem;

    /**
     * Contains the main SwerveModule logic of the bot
     * @param rID is a CAN ID parameter(int)
     * @param sID is a CAN ID parameter(int)
     * @param eID is a CAN ID parameter(int)
     * @param absolutePositionAtRobotZero is absolute pos at zero in deg (double)
     * @param module for numbering modules during comprehensive shuffleboard outputs
     */
    public SwerveModule(int rID, int sID, int eID, double absolutePositionAtRobotZero, double offset,int module) {
        this.rID = rID;
        this.eID = eID;
        this.sID = sID;
        this.module = module;
        this.offset = offset;
        cancoderOffset = -absolutePositionAtRobotZero;

        turnMotor = new CANSparkMax(rID, MotorType.kBrushless); //assuming two NEOs
        speedMotor = new CANSparkMax(sID, MotorType.kBrushless);
        turnEncoder = new CANCoder(eID);
        driveEncoder = speedMotor.getEncoder();
        angleController = new PIDController(0.003, 0, 0.00000); //never changes after initialization anyways

        config();
    }

    public void config() {
        turnMotor.setIdleMode(IdleMode.kCoast);
        speedMotor.setIdleMode(IdleMode.kCoast);

        angleController.setTolerance(0.5); //degrees for now...
        angleController.enableContinuousInput(0, 360); //in accordance to rotation2D default degrees format

        turnEncoder.configFactoryDefault();
        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360); //SushiSquad and Fusion prefer 0 to 360 but whatever
        turnEncoder.configMagnetOffset(cancoderOffset);
        turnEncoder.configSensorDirection(false);
        resetToAbsolute();
    }
    private void resetToAbsolute() {
        double absolutePosition = getTurningPosition() - cancoderOffset;
        turnEncoder.setPosition(absolutePosition);
        driveEncoder.setPosition(absolutePosition);

      }

    public void SwerveCoast() {
        turnMotor.setIdleMode(IdleMode.kCoast);
        speedMotor.setIdleMode(IdleMode.kCoast);
    }
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }
  

    /**
    * Without worrying about factors such as  field-centric drive, chassis conversion, etc, this is the most fundamental method of controlling each swerve module
    * @param speed in m/s (to match with wpilib's format) which is later converted
    * @param rotation2D angle in wpilib's Rotation2D object format
    */
    public void setSwerveModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            resetToAbsolute();
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        speedMotor.set(state.speedMetersPerSecond / Constants.SwerveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(angleController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void setSwerveModuleState2(SwerveModuleState state) {
        setSpeed(state.speedMetersPerSecond);
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
        //set refers to percentage motor speed output. Internally it controls voltage (which is surprisingly closely proportional to rpm) and uses a type of setpoint command
        double motorInput = speed/Constants.SwerveConstants.maxTranslationalSpeed;
        SmartDashboard.putNumber("Speed" + ((Integer) module), motorInput);
        speedMotor.set(motorInput);
    }
    public void stop() {
        resetToAbsolute();
        speedMotor.set(0);
        turnMotor.set(0);
      }
    

    /**
    * Rotates the bot to the specified angle with precision
    * @param rotation2D is of type Rotation2d. This is the angle that you want to turn the robot
    */
    public void setAnglePID(Rotation2d rotation2D) {   
        double angleSetpoint = rotation2D.getDegrees(); // 0 to 360!
        // angleSetpoint = MathUtil.inputModulus(angleSetpoint, -180, 180);
        SmartDashboard.putNumber("Angle Setpoint" + ((Integer) module), angleSetpoint);
        SmartDashboard.putNumber("Current Angle" + ((Integer) module), getAngle());
        SmartDashboard.putNumber("Position Errror" + ((Integer) module), angleSetpoint - getAngle());

        // if (angleController.atSetpoint()) {
        //     return;
        // }
        //We should clamp the PID output to between -1 and 1
        double PIDVAL = angleController.calculate(getAngle(), angleSetpoint);
        double PIDVALCLAMP = MathUtil.clamp(PIDVAL , -0.3 , 0.3);
        SmartDashboard.putNumber("PIDVAL" + ((Integer) module), PIDVAL);
        SmartDashboard.putNumber("PIDVALCLAMP" + ((Integer)module), PIDVALCLAMP);

        turnMotor.set(-PIDVALCLAMP);
    }
    
    /**
     * Gets the absolute position of the cancoder, it tells you the angle of rotation of CANCoder. 
     * @return the angle of the bot from the original starting angle
     */
    public double getAngle() {
        // return turnEncoder.getAbsolutePosition() - offset; //make sure this is degrees
        return (turnEncoder.getAbsolutePosition()); //make sure this is degrees
    }
    
  }