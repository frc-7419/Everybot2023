package frc.robot.subsystems.vacuum;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PowerConstants;

public class RunVacuum extends CommandBase {
  private VacuumSubsystem vacuumSubsystem;

  public RunVacuum(VacuumSubsystem vacuumSubsystem) {
    this.vacuumSubsystem = vacuumSubsystem;
    addRequirements(vacuumSubsystem);
  }

  @Override
  public void initialize() {
    vacuumSubsystem.setPower(PowerConstants.VacuumPower);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    vacuumSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
