package frc.robot.subsystems.groundIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunGroundIntake extends CommandBase {

  private GroundIntakeSubsystem groundIntakeSubsystem;
  private double power;
  
  public RunGroundIntake(GroundIntakeSubsystem groundIntakeSubsystem, double power) {
    this.groundIntakeSubsystem = groundIntakeSubsystem;
    this.power = power;
    addRequirements(groundIntakeSubsystem);
  }

  @Override
  public void initialize() {
    groundIntakeSubsystem.coast();
    groundIntakeSubsystem.setSpeed(power);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    groundIntakeSubsystem.setSpeed(0);
    groundIntakeSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
