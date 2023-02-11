// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

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

    public static enum CanIds {
        // 2020 drive train ids
        leftFalcon1(1),
        rightFalcon1(2),
        leftFalcon2(4),
        rightFalcon2(3),
        
        //Can ids need to be found and added for intake + arm
        ;
        
        public final int id;

        private CanIds(int id) {
            this.id = id;
        }
        
    }
    public static class RobotConstants {
        public static final double TalonFXTicksPerRotation = 2048;

        public static final double kTrackWidth = 0.6858; // meters

        public static final double kWheelRadius = 3 * 0.0254; // We use Hi-Grip 6 inch wheels so convert to meters
        public static final double kWheelCircumference = 2 * Math.PI * Constants.RobotConstants.kWheelRadius;

        public static final double timeStep = 0.2; //how often periodic() function runs

    }
    public static class GearConstants {
        public static final double ToughboxMiniRatio = (double) (50.0) / 14.0 * 45.0 / 19.0;
        /*2 14tooth pinions mate to a 50 tooth gear,
         above which a 19 tooth gear mates to a 45 tooth gear
         In total around 8.4:1
        */
    }

    public static class PowerConstants {
        public static final double DriveBaseStraight = .55;
        public static final double DriveBaseTurn = .35;
        public static final double IntakePower = 1.0; //arbitrary for now
        public static final double ArmPower = 0.6;//arbitrary for now
        public static double autoDockPower;
    }

    public static final Port SerialPortAHRS = null;

    public static class FieldConstants {
        public static final double length = Units.feetToMeters(54);
        public static final double width = Units.feetToMeters(27);
    }

    public static class PIDConstants {
        public static final double turnWithAprilTagkP = 0.005;
        public static final double turnWithAprilTagkI = 0;
        public static final double turnWithAprilTagkD = 0;

    }

    
    
}