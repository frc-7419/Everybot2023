package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntakeWithJoystick extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private XboxController joystick;

  public RunIntakeWithJoystick(XboxController joystick, IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.joystick = joystick;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.coast();
  }

  @Override
  public void execute() {
    if (joystick.getLeftBumper()) {
      intakeSubsystem.coast();
      intakeSubsystem.setSpeed(-0.5);
      // SmartDashboard.putNumber("Arm Power", joystick.getLeftY());
    } else if (joystick.getRightBumper()) {
      intakeSubsystem.coast();
      intakeSubsystem.setSpeed(0.5);
    } else {
      // TODO: Replace the 0 with some tested constant that keeps the arm stationary -
      // PowerConstants.armStationaryPower
      intakeSubsystem.setSpeed(0);
      intakeSubsystem.brake();
    }
  }

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
