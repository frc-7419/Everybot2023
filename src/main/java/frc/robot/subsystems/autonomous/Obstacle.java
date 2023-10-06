package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.Arrays;

public class Obstacle {
    public static final double BUFFER = .3;

    public Translation2d lowerLeft;
    public Translation2d upperRight;

    public Obstacle(Translation2d lowerLeftCorner, Translation2d upperRightCorner) {
        double[] x = new double[]{lowerLeftCorner.getX(), upperRightCorner.getX()};
        double[] y = new double[]{lowerLeftCorner.getY(), upperRightCorner.getY()};

        this.lowerLeft = new Translation2d(Arrays.stream(x).min().getAsDouble() - BUFFER, Arrays.stream(y).min().getAsDouble() - BUFFER);
        this.upperRight = new Translation2d(Arrays.stream(x).max().getAsDouble() + BUFFER, Arrays.stream(y).max().getAsDouble() + BUFFER);
    }
}