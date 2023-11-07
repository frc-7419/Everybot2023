
package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX arm;
  private final TrapezoidProfile.Constraints constraints;
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(CanIds.armEncoder.id);
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State start;  
  /**
   * Makes a new ArmSubsystem
   */
  public ArmSubsystem() {
    constraints = new TrapezoidProfile.Constraints(ArmConstants.maxVelocity, ArmConstants.maxAcceleration);
    start = new TrapezoidProfile.State(0, 0);
    arm = new TalonFX(CanIds.arm.id);
    encoder.setPositionOffset(0.144);
    // arm.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,20,20,0.5));
    // arm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,20,20,0.5));

    arm.configVoltageCompSaturation(11);
    arm.enableVoltageCompensation(true);
  }
  /**
   * Sets the power to the motor which moves the arm back and forth
   * @param power in percent
   */
  public void setPower(double power) {
    arm.set(ControlMode.PercentOutput, power);
  }
  /**
   * sets the mode of the motor to coast
   */
  public void coast() {
    arm.setNeutralMode(NeutralMode.Coast);
  }
  /**
   * sets the mode of the motor to brake
   */
  public void brake() {
    arm.setNeutralMode(NeutralMode.Brake);
  }
  /**
   * gets the position of the arm from the absolute encoder minus the offset
   * @return
   */
  public double getPosition() {
    return encoder.getAbsolutePosition() - encoder.getPositionOffset();
  }

  public TrapezoidProfile.Constraints getConstraints() {
    return constraints;   
  }
  public TrapezoidProfile.State getStart() {
    return start;
  }
  public TrapezoidProfile.State getGoal() {
    return goal;
  }
  public void setGoal(double goal){
    this.goal = new TrapezoidProfile.State(goal, 0);
  }
  public void setSetpoint(TrapezoidProfile.State nextSetpoint) {
    start = nextSetpoint;
  }
  /**
   * checks if it is connected
   * @return if the encoder is connected
   */
  public boolean isConnected() {
    return encoder.isConnected();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Absolute Encoder Angle", getPosition());
    SmartDashboard.putBoolean("connected", encoder.isConnected());
  }
}
