// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // public static enum CanIds {

    //     // 2020 drive train ids
        
    //     //Can ids need to be found and added for intake + arm
    //     // leftFalcon1(62),
    //     // driveLeft1(3),
    //     // driveLeft2(4),
    //     // driveRight1(1),
    //     // driveRight2(2)
        
        
    //     public final int id;

    //     private CanIds(int id) {
    //         this.id = id;
    //     }
        
    // }

    // public static class CanIdsNoEnum {

    //     public static final int leftFalcon1 = CanIds.leftFalcon1.id;
    //     public static final int driveLeft1 = CanIds.driveLeft1.id;
    //     public static final int driveLeft2 = CanIds.driveLeft2.id;
    //     public static final int driveRight1 = CanIds.driveRight1.id;
    //     public static final int driveRight2 = CanIds.driveRight2.id;

    // }

    public static class Swerve {
        public static final double swerveKinematics = 0.0; // TODO: Needs to be changed to accurate swerve Kinematics


        public static final double pigeonID = 0.0;
    }
    
    public static class RobotConstants {

        public static final double TalonFXTicksPerRotation = 2048;

        public static final double kTrackWidth = 0.6858; // meters

        public static final double kWheelRadius = 3 * 0.0254; // We use Hi-Grip 6 inch wheels so convert to meters
        public static final double kWheelCircumference = 2 * Math.PI * Constants.RobotConstants.kWheelRadius;

        public static final double timeStep = 0.2; //how often periodic() function runs
        public static final double LENGTH = Units.inchesToMeters(26.5);
        public static final double WIDTH = Units.inchesToMeters(26.5);

    }
    public static class GearConstants {


    }

    public static class ArmConstants {

        public static final double kP = 0.0001;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kTolerance = 100;

    }

    public static class PowerConstants {
        public static final double IntakePower = 0.7; //arbitrary for now
        public static final double ArmPower = 0.2;//arbitrary for now
    }

    public static class DriveConstants{

    }

    public static class AngleOffset {

        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(37.35);

    }

    public static final Port SerialPortAHRS = null;
    
    /**
     * THIS IS FOR THE SWERVE DRIVE CONSTANTS
     */

    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


  
  public static class SwerveConstants {
    //Not sure how to calculate this theoretically but this needs to be determined experimentally first
    public static double maxSpeed = 2.0;
    /*
    * IMPORTANT: THIS WAS FOUND THROUGH CAD FILES BUT THERE ARE MANY SWERVE X CONFIGURATIONS
    * SO YOU NEED TO DOUBLE CHECK THIS IS CORRECT IN PRACTICE
    */
    /* ANGLE MOTOR
    * NEO Shaft to 12T Pulley to 24T Pulley to 14T Gear to 72T Main Rotation Gear
    */
    public static double gearRatioAngleMotor = (double) 12.0/24.0*14.0/72.0;
    /* DRIVE MOTOR
     * NEO shaft to 12T Pulley to 24T Pulley to 24T Gear to 22T Gear to 15T bevel to 45T Bevel
     *
     * The CANCODER measures rotations of a the driven 1:1 PULLEY in which the driver pulley is on the same
     * shaft as the 24T Pulley
     */
    public static double gearRatioSpeedMotor = (double) 12.0/24.0* 24.0/22.0 * 15.0/45.0;
    /*
     * So Number of Rotations of this CANCOder sensor measured means this amount of rotations in actual SPEED wheel
     */
    public static double gearRatioCANCoder = (double) 24.0/22.0 * 15.0/45.0;
    public static double wheelDiameter = Units.inchesToMeters(4.0);
    public static double wheelCircumfrence = wheelDiameter * 2 * Math.PI;
    public static final double anglekP = 0.6;
    //TO-DO set the correct turnEncoder IDs
    //TO-DO instead of swerve0, swerve1, swerve2, and swerve3 set the correct name like frontleft, frontright, backleft, etc.. 
    public static final SwerveModuleConstants swerve0 = new SwerveModuleConstants(
      2, 1, 0, new Translation2d(RobotConstants.LENGTH, RobotConstants.LENGTH) );
    public static final SwerveModuleConstants swerve1 = new SwerveModuleConstants(
      4, 3, 0, new Translation2d(RobotConstants.LENGTH, -RobotConstants.LENGTH));
    public static final SwerveModuleConstants swerve2 = new SwerveModuleConstants(
      6, 5, 0, new Translation2d(-RobotConstants.LENGTH, RobotConstants.LENGTH));
    public static final SwerveModuleConstants swerve3 = new SwerveModuleConstants(
      8, 7, 0, new Translation2d(-RobotConstants.LENGTH, -RobotConstants.LENGTH));
  }
  public static class SwerveModuleConstants {
    public int speedMotorID;
    public int turnMotorID;
    public int turnEncoderID;
    public Translation2d location;
    public SwerveModuleConstants(int speedMotorID, int turnMotorID, int turnEncoderID, Translation2d location) {
      this.speedMotorID = speedMotorID;
      this.turnMotorID = turnMotorID;
      this.turnEncoderID = turnEncoderID;
      this.location = location;
    }
  }


}