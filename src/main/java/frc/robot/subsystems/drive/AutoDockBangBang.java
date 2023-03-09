package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PowerConstants;

public class AutoDockBangBang extends CommandBase {
  private DriveBaseSubsystem driveBaseSubsystem;
  private final double PITCH_SETPOINT = 0;

  public AutoDockBangBang(DriveBaseSubsystem driveBaseSubsystem) {
    this.driveBaseSubsystem = driveBaseSubsystem;
  }

  @Override
  public void initialize() {
    driveBaseSubsystem.coast();
  }

  @Override
  public void execute() {
    double pitch = driveBaseSubsystem.getPitch();

    double output = PowerConstants.autoDockPower;

    if (pitch < PITCH_SETPOINT) {
      output *= -1;
    }

    driveBaseSubsystem.setAllPower(output);
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.setAllPower(0);
    driveBaseSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return false;
    // return gyroSubsystem.getPitch() == PITCH_SETPOINT;
  }
}
