package frc.robot.subsystems.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;

public class FollowTrajectory extends SequentialCommandGroup {
  public FollowTrajectory(DriveBaseSubsystem driveBaseSubsystem, PathPlannerTrajectory traj, boolean isFirstPath) {
    addCommands(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            driveBaseSubsystem.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj,
            driveBaseSubsystem::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA),
            driveBaseSubsystem.kinematics, // DifferentialDriveKinematics
            driveBaseSubsystem::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
            driveBaseSubsystem::outputVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            driveBaseSubsystem // Requires this drive subsystem
        ),
        new InstantCommand(() -> {
          driveBaseSubsystem.setAllPower(0);
        })
      );
  }
}
