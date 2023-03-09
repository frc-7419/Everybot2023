package frc.robot;

public final class Constants {

    public static enum CanIds {
        leftFalcon1(62),
        driveLeft1(3),
        driveLeft2(4),
        driveRight1(1),
        driveRight2(2);

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

        public static final double timeStep = 0.2; // how often periodic() function runs
    }

    public static class GearConstants {
        public static final double ToughboxMiniRatio = (double) (50.0) / 14.0 * 45.0 / 19.0;
        /*
         * 2 14tooth pinions mate to a 50 tooth gear,
         * above which a 19 tooth gear mates to a 45 tooth gear
         * In total around 8.4:1
         */
    }

    public static class DriveConstants {
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
    }

    public static class ArmConstants {
        public static final double kP = 0.0001;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kTolerance = 100;
    }

    public static class PowerConstants {
        public static final double DriveBaseStraight = .55;
        public static final double DriveBaseTurn = .35;
        public static final double IntakePower = 0.7; // arbitrary for now
        public static final double ArmPower = 0.2;// arbitrary for now
        public static double autoDockPower = 0.2;
    }

};