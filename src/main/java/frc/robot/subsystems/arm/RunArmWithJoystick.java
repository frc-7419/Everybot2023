package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PowerConstants;

public class RunArmWithJoystick extends CommandBase {
  private final XboxController joystick;
  private final ArmSubsystem armSubsystem;
  /**
   * Runs the arm using an Xbox joystick left Y axis
   * @param joystick
   * @param armSubsystem
   */
  public RunArmWithJoystick(XboxController joystick, ArmSubsystem armSubsystem) {
    this.joystick = joystick;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.coast(); //not sure why coast at init but 7419 did so for elevator
  }

  @Override
  public void execute() {
    if (Math.abs(joystick.getLeftY()) > 0.05) {
      armSubsystem.coast();
      double power = joystick.getLeftY() * PowerConstants.maxArmPower;
      armSubsystem.setPower(power);
      
      SmartDashboard.putNumber("Arm Power", power);
    }
    else {
      //TODO: Replace the 0 with some tested constant that keeps the arm stationary - PowerConstants.armStationaryPower
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