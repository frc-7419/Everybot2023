package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmWithPID extends CommandBase {
  ArmSubsystem armSubsystem;
  PIDController pidController;
  double setpoint = 0;
  double tolerance = 0.5;
  double kP = 0;
  double kI = 0;
  double kD = 0;

  public ArmWithPID(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    this.pidController = new PIDController(kP, kI, kD);
    addRequirements(armSubsystem);
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("setpoint", setpoint);
  }

  @Override
  public void initialize() {
    kP = SmartDashboard.getNumber("kP", kP);
    kI = SmartDashboard.getNumber("kI", kI);
    kD = SmartDashboard.getNumber("kD", kD);
    setpoint = SmartDashboard.getNumber("setpoint", setpoint);

    pidController = new PIDController(
        kP, kI, kD);
    pidController.setTolerance(tolerance);
    pidController.setSetpoint(setpoint);

    armSubsystem.coast();
  }

  @Override
  public void execute() {
    double position = armSubsystem.getPosition();
    double output = pidController.calculate(position);
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
