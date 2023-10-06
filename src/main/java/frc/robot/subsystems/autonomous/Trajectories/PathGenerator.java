// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autonomous.Trajectories;

import java.util.ArrayList;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.FieldConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.autonomous.Obstacle;
import frc.robot.autonomous.ObstacleConstants;
import frc.robot.util.PriorityQueue;

/**
 * Add your docs here.
 */
public class PathGenerator {
    private final Pose2d initialPosition;
    private final Pose2d finalPosition;
    private ArrayList<Translation2d> controlPoints;

    public PathGenerator(Pose2d initialPose, Pose2d lastPose) {
        this.initialPosition = FieldConstants.allianceFlip(initialPose);
        this.finalPosition = FieldConstants.allianceFlip(lastPose);
        this.controlPoints = buildPath(astar(new Translation2d(this.initialPosition.getX(), this.initialPosition.getY()), new Translation2d(this.finalPosition.getX(), this.finalPosition.getY())));
        removeDuplicateSlopes();
        prunePath();
    }

    private static class PathNode {
        Translation2d position;
        Translation2d finalPosition;
        PathNode parent;

        public PathNode(Translation2d position, Translation2d finalPosition) {
            this.position = position;
            this.finalPosition = finalPosition;
        }
    }

    private boolean containedIn(Translation2d pos, Translation2d lowerLeft, Translation2d upperRight) {
        return pos.getX() >= lowerLeft.getX() && pos.getY() >= lowerLeft.getY() && pos.getX() <= upperRight.getX() && pos.getY() <= upperRight.getY();
    }

    private boolean inObstacle(Translation2d pos) {
        for (Obstacle obstacle : ObstacleConstants.obstacleList) {
            if (containedIn(pos, obstacle.lowerLeft, obstacle.upperRight)) {
                return true;
            }
        }

        return false;
    }

    private boolean obstacleBetween(Translation2d initialPos, Translation2d finalPos) {
        final int steps = 25;
        Translation2d step = new Translation2d((finalPos.getX() - initialPos.getX()) / steps, (finalPos.getY() - initialPos.getY()) / steps);

        for (int i = 0; i <= steps; i++) {
            if (inObstacle(initialPos)) {
                return true;
            }
            initialPos = initialPos.plus(step);
        }

        return false;
    }

    private ArrayList<PathNode> getNeighbors(PathNode node, Translation2d finalPosition) {
        ArrayList<PathNode> neighbors = new ArrayList<>();

        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                if (x == y) continue;
                Translation2d pos = new Translation2d(node.position.getX() + (double) x / 4, node.position.getY() + (double) y / 4);
                if (!inObstacle(pos)) {
                    PathNode element = new PathNode(pos, finalPosition);
                    neighbors.add(element);
                }
            }
        }

        return neighbors;
    }

    private PathNode astar(Translation2d initialPosition, Translation2d finalPosition) {
        PriorityQueue<PathNode> frontier = new PriorityQueue<>();
        frontier.add(new PathNode(initialPosition, finalPosition), 0);
        ArrayList<Translation2d> visited = new ArrayList<>();

        while (!frontier.isEmpty()) {
            PathNode currentNode = frontier.remove();
            if (Math.abs(currentNode.position.getDistance(finalPosition)) < 1) {
                return currentNode;
            }

            for (PathNode child : getNeighbors(currentNode, finalPosition)) {
                if (!visited.contains(child.position)) {
                    visited.add(child.position);
                    child.parent = currentNode;
                    frontier.add(child, Math.abs(child.position.getDistance(finalPosition)) + Math.abs(child.position.getDistance(initialPosition)));
                }
            }
        }

        return null;
    }

    private ArrayList<Translation2d> buildPath(PathNode finalNode) {
        ArrayList<Translation2d> path = new ArrayList<>();

        PathNode currentNode = finalNode;
        while (currentNode.parent != null) {
            path.add(0, currentNode.position);
            currentNode = currentNode.parent;
        }

        return path;
    }

    private double getSlope(Translation2d first, Translation2d second) {
        return (first.getY() - second.getY()) / (first.getX() - second.getX());
    }

    private void removeDuplicateSlopes() {
        if (controlPoints.size() < 2) {
            return;
        }
        ArrayList<Translation2d> newPath = new ArrayList<>();
        double lastSlope = getSlope(initialPosition.getTranslation(), controlPoints.get(1));

        for (int i = 0; i < controlPoints.size() - 1; i++) {
            double currentSlope = getSlope(controlPoints.get(i), controlPoints.get(i + 1));
            if (currentSlope != lastSlope) {
                newPath.add(controlPoints.get(i));
            }
            lastSlope = currentSlope;
        }

        controlPoints = newPath;
    }

    private void prunePath() {
        for (int i = controlPoints.size() - 1; i >= 0; i--) {
            if (!obstacleBetween(initialPosition.getTranslation(), controlPoints.get(i))) {
                controlPoints.subList(0, i).clear();
                break;
            }
        }

        for (int i = 0; i < controlPoints.size(); i++) {
            if (!obstacleBetween(finalPosition.getTranslation(), controlPoints.get(i))) {
                controlPoints.subList(i + 1, controlPoints.size()).clear();
                break;
            }
        }
    }

    public final Translation2d[] getPointList() {
        ArrayList<Translation2d> points = (ArrayList<Translation2d>) controlPoints.clone();
        points.add(0, initialPosition.getTranslation());
        points.add(finalPosition.getTranslation());


        return (Translation2d[]) points.toArray();
    }


    public final Trajectory getTrajectory() {
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                10);

        TrajectoryConfig config = new TrajectoryConfig(
                TeleopConstants.kMaxSpeedMetersPerSecond,
                TeleopConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        return TrajectoryGenerator.generateTrajectory(initialPosition, controlPoints, finalPosition, config);
    }
}
