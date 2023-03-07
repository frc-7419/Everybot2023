package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PowerConstants;

public class RunIntakeWithJoystick extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private XboxController joystick;
  public RunIntakeWithJoystick(IntakeSubsystem intakeSubsystem, XboxController joystick) {
    this.intakeSubsystem = intakeSubsystem;
    this.joystick = joystick;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.setPower(0);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Intake Power", intakeSubsystem.getPower());
    if (joystick.getLeftTriggerAxis() > 0) {
      intakeSubsystem.coast();
      intakeSubsystem.setPower(PowerConstants.IntakePower);
    }
    else if (joystick.getRightTriggerAxis() > 0) {
      intakeSubsystem.coast();
      intakeSubsystem.setPower(-PowerConstants.IntakePower);
    }
    else {
      intakeSubsystem.setPower(0);
      intakeSubsystem.brake();
    }
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
