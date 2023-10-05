package frc.robot.commands.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBaseSubsystem;

public class FollowTrajectory extends CommandBase {
    private DriveBaseSubsystem driveBase;
    private String path;

    public FollowTrajectory(DriveBaseSubsystem drivebase, String path) {
        this.driveBase = drivebase;
        this.path = path;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // fix PathConstraints with the actual constraints that we want to use with the robot
        PathPlannerTrajectory examplePath = PathPlanner.loadPath(this.path, new PathConstraints(4, 3));
        driveBase.followTrajectoryCommand(examplePath, false, driveBase);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public void setPath(String path) {
        this.path = path;
    }
}
