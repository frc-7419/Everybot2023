package frc.robot;

public final class Constants {

    public static enum CanIds {
        leftMast(5),
        rightMast(2),
        leftFollow(4),
        rightFollow(3),
        // Can ids need to be found and added for intake + arm
        arm(4),
        vacuum(1),
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
    }

    public static class PowerConstants {
        public static final double DriveBaseStraight = 1;
        public static final double DriveBaseTurn = 1;
        public static final double IntakePower = 1.0; // arbitrary for now
        public static final double ArmPower = 0.6;// arbitrary for now
        public static final double VacuumPower = 0.7;
    }
};