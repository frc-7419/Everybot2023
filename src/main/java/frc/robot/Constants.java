// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  public static class RobotConstants {
      public static final double LENGTH = Units.inchesToMeters(26.5);
      public static final double HALF_LENGTH = LENGTH/2.0;
  }
  public static class SwerveConstants {
    public static final double kDeadband = 0.05;

    public static final int kLeftJoystick = 0;
    public static final int kRightJoystick = 1;
    public static final int kXboxController = 3;

    public static final int kFieldOrientedButton = 3;
    public static final int kZeroHeadingButton = 2;
    public static final int kRotatorButton = 3;
    public static final double kTransmitterOffset = 1.429;
            // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(21.25);

        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(21.25);

        // Need to update to correct values, I dont remember the value we set last meet
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //fl
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //fr
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //bl
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //br


        /*
         * 
         * 
         * new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //br
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //fr
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //bl
                new Translation2d(kWheelBase / 2, kTrackWidth / 2)); //fl

         */

                                                               // Driving Motor Ports
        public static final int kFrontLeftDriveMotorPort = 2;  // Front Left 
        public static final int kFrontRightDriveMotorPort = 3; // Front Right
        public static final int kBackRightDriveMotorPort = 5;  // Back Right
        public static final int kBackLeftDriveMotorPort = 7;   // Back Left

                                                                // Turning Motor Ports
        public static final int kFrontLeftTurningMotorPort = 1; // Front Left
        public static final int kFrontRightTurningMotorPort = 4;// Front Right
        public static final int kBackRightTurningMotorPort = 6; // Back Right
        public static final int kBackLeftTurningMotorPort = 8;  // Back Left

        // Encoder on NEO turning
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        // Encoder for NEO drive
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true; //
        public static final boolean kBackRightDriveEncoderReversed = true;  //

        // -------> ABE <-------- //
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 12;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
        public static final int kBackRightDriveAbsoluteEncoderPort = 11;
        // -------> ABE <-------- //

        // Absolute encoders reversed
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

                                        // Need to update values for our specific magnetic fields
                                        // NOT IN RADIANS!
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.658063;//0.98 * 2 * Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 1.4;        // 1.04719      //1.04719
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.39626;//(0.0141+.25) * 2 * Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  -0.122173;//0.2577 * 2 * Math.PI;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 8; //1.75
       // public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5; //1.75
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2; //2

        public static final double kPThetaController = 0.001;
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.00;
        
        public static final double kMaxDriveMotorTemp = 33.0;
   
  }
  public static class SwerveModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 8.14 ; 
    public static final double kTurningMotorGearRatio = 1 / 12.8; 
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.005;
    }
  };