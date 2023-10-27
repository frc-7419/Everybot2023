
package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX arm;
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(CanIds.armEncoder.id);
  /**
   * Makes a new ArmSubsystem
   */
  public ArmSubsystem() {
    arm = new TalonFX(CanIds.arm.id);
    encoder.setPositionOffset(0.144);
    arm.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,20,20,0.5));
    arm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,20,20,0.5));
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
