package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotConstants;

public class ArmWithPIDProfile extends CommandBase {
  ProfiledPIDController controller;
  ArmSubsystem armSubsystem;
  double kP = 0.001;
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0, 0);

  public ArmWithPIDProfile(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    controller = new ProfiledPIDController(kP, 0, 0, null, RobotConstants.loopDt);
    controller.setGoal(2);
  }

  @Override
  public void execute() {
    armSubsystem.setPower(controller.calculate(armSubsystem.getPosition()));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
