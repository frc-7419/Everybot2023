
package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBaseSubsystem;

public class drive extends CommandBase {
  /** Creates a new findDock. */
  private DriveBaseSubsystem driveBaseSubsystem;
  private Pose2d dest;
  private Pose2d curr;

  public drive(DriveBaseSubsystem driveBaseSubsystem, Pose2d pose2d) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    dest = pose2d;
    curr = driveBaseSubsystem.getPose();
    addRequirements(driveBaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // we want the robot to drive to the specific dest pose given the curr pose as input
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
