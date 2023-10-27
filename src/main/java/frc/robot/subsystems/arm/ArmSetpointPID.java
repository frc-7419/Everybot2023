package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmSetpointPID extends CommandBase {
  private ArmSubsystem armSubsystem;
  private PIDController pidController;
  double setpoint;
  double tolerance = 0.01; // When the error is less than the tolerance the PID stops
  /**
   * This command is for setpoints, it uses a PID controller to move the arm to the given setpoint
   * @param armSubsystem
   * @param setpoint
   */
  public ArmSetpointPID(ArmSubsystem armSubsystem, double setpoint) {
    this.setpoint = setpoint;
    this.armSubsystem = armSubsystem;
    this.pidController = new PIDController(0.5, 0, 0);
    addRequirements(armSubsystem);
  }
  
  @Override
  public void initialize() {
    pidController.setTolerance(tolerance);
    pidController.setSetpoint(setpoint);
    armSubsystem.coast();
  }

  @Override
  public void execute() {
    double position = armSubsystem.getPosition();
    double output = MathUtil.clamp(pidController.calculate(position), -0.5, 0.5);
    SmartDashboard.putNumber("PID output", output);
    SmartDashboard.putNumber("PID error", position-setpoint);
    armSubsystem.setPower(-output);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setPower(0);
    armSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
