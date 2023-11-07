package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
//
// THIS IS FOR TESTING AND TUNING PID FOR ArmSetPointPID command
// setpoints are set on dashboard for this command
//
public class ArmWithPID extends CommandBase {
  private ArmSubsystem armSubsystem;
  private PIDController pidController;
  double setpoint = 0;
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kF = 0;

  public ArmWithPID(ArmSubsystem armSubsystem, double kP, double kF, double setpoint) {
    this.armSubsystem = armSubsystem;
    this.kP = kP;
    this.kF = kF;
    this.setpoint = setpoint;
    this.pidController = new PIDController(kP, kI, kD);
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    pidController = new PIDController(
        kP, kI, kD);
    pidController.setTolerance(0);
    pidController.setSetpoint(setpoint);

    armSubsystem.coast();
  }

  @Override
  public void execute() {
    double position = armSubsystem.getPosition();
    double output = MathUtil.clamp(pidController.calculate(position), -0.5, 0.5);
    armSubsystem.setPower(-output-kF);
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