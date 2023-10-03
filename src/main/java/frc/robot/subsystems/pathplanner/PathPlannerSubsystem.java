package frc.robot.subsystems.pathplanner;


import java.util.function.Consumer;
import java.util.function.Supplier;


import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drive.SwerveModule;




public class PathPlannerSubsystem {


    private Supplier<Pose2d> getPose;
    private Consumer<ChassisSpeeds> outputChassisSpeeds;
    private SwerveModule[] swerveModules;

    private SwerveModulePosition[] positions;


    public PathPlannerSubsystem() {
        swerveModules = new SwerveModule[] {
            new SwerveModule(SwerveConstants.swerve0.turnMotorID, SwerveConstants.swerve0.speedMotorID, SwerveConstants.swerve0.turnEncoderID),
            new SwerveModule(SwerveConstants.swerve1.turnMotorID, SwerveConstants.swerve1.speedMotorID, SwerveConstants.swerve1.turnEncoderID),
            new SwerveModule(SwerveConstants.swerve2.turnMotorID, SwerveConstants.swerve2.speedMotorID, SwerveConstants.swerve2.turnEncoderID),
            new SwerveModule(SwerveConstants.swerve3.turnMotorID, SwerveConstants.swerve3.speedMotorID, SwerveConstants.swerve3.turnEncoderID),
        };


        positions = new SwerveModulePosition[4];
        positions[0] = getSwerveModule(0).getPosition();
        positions[1] = getSwerveModule(1).getPosition();
        positions[2] = getSwerveModule(2).getPosition();
        positions[3] = getSwerveModule(3).getPosition();
    }

    public SwerveModule getSwerveModule(int index) {
        return swerveModules[index];
    }



    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, DriveBaseSubsystem driveBaseSubsystem) {


        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                if(isFirstPath){
                    this.resetOdometry(traj.getInitialPose());
                }
            }),
            new PPSwerveControllerCommand(
                traj,
                getPose, // Pose supplier
                new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                outputChassisSpeeds,
                driveBaseSubsystem // Requires this drive subsystem
            )
        );
    }


    private void resetOdometry(Pose2d initialPose) {


    }
}



