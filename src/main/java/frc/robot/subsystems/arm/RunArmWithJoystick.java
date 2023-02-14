package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunArmWithJoystick extends CommandBase {
  private ArmSubsystem armSubsystem;
  private XboxController joystick;

  public RunArmWithJoystick(ArmSubsystem armSubsystem, XboxController joystick) {
    this.armSubsystem = armSubsystem;
    this.joystick = joystick;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armSubsystem.setPower(joystick.getRightY());
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
