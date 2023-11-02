package frc.robot.subsystems.arm;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
public class ArmSetpointPID extends CommandBase {
  private ArmSubsystem armSubsystem;
  private PIDController pidController;
  private TrapezoidProfile currentProfile;
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
    armSubsystem.setGoal(setpoint);
    double armPosition = armSubsystem.getPosition();
    pidController.setTolerance(tolerance);
    pidController.setSetpoint(setpoint);
    armSubsystem.coast();
  }

  @Override
  public void execute() {
    currentProfile = new TrapezoidProfile(armSubsystem.getConstraints(), armSubsystem.getGoal(), armSubsystem.getStart());
    
    TrapezoidProfile.State nextSetpoint = currentProfile.calculate(0.01);

    armSubsystem.setSetpoint(nextSetpoint);
    pidController.setSetpoint(nextSetpoint.position);
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
