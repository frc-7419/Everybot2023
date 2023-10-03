package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPosition extends CommandBase {
  private ArmSubsystem armSubsystem;
  private PIDController pidController;
  private double setpoint;

  public ArmToPosition(ArmSubsystem armSubsystem, double setpoint) {
    this.armSubsystem = armSubsystem;
    this.setpoint = setpoint;
    SmartDashboard.putNumber("Arm Setpoint", setpoint);
    SmartDashboard.putNumber("Arm kP", ArmConstants.kP);
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    double kP = ArmConstants.kP;
    kP = SmartDashboard.getNumber("Arm kP", kP);
    pidController = new PIDController(kP, 0, 0);
    pidController.setTolerance(ArmConstants.kTolerance);
    setpoint = SmartDashboard.getNumber("Arm Setpoint", setpoint);
    armSubsystem.coast();
  }

  @Override
  public void execute() {
    double output = pidController.calculate(armSubsystem.getPosition());
    armSubsystem.setPower(output);
    SmartDashboard.putNumber("Arm PID Output", output);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setPower(0);
    armSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return false;
    // return pidController.atSetpoint();
  }
}