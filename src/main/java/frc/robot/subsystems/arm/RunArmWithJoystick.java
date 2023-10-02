package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunArmWithJoystick extends CommandBase {
  private final XboxController joystick;
  private final ArmSubsystem armSubsystem;
  public RunArmWithJoystick(XboxController joystick, ArmSubsystem armSubsystem) {
    this.joystick = joystick;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SmartDashboard.putNumber("Arm Joystick Power", joystick.getLeftY());
    if (Math.abs(joystick.getLeftY()) > 0.05) {
      armSubsystem.coast();
      armSubsystem.setPower(joystick.getLeftY() * 0.25);
    }
    else {
      armSubsystem.setPower(0);
      armSubsystem.brake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setPower(0);
    armSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}