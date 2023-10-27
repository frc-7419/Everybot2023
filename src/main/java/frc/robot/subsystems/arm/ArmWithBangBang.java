package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmWithBangBang extends CommandBase {
  private ArmSubsystem armSubsystem;

  public ArmWithBangBang(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
