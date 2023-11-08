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
    groundIntakeSubsystem.intakeCoast();
    groundIntakeSubsystem.setIntakeSpeed(power);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    groundIntakeSubsystem.setIntakeSpeed(0);
    groundIntakeSubsystem.intakeBrake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
