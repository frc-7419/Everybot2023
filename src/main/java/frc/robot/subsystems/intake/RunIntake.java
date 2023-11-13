package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntake extends CommandBase {

  private IntakeSubsystem intakeSubsystem;
  private double power;
  
  public RunIntake(IntakeSubsystem intakeSubsystem, double power) {
    this.intakeSubsystem = intakeSubsystem;
    this.power = power;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.coast();
    intakeSubsystem.setSpeed(power);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setSpeed(0);
    intakeSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
