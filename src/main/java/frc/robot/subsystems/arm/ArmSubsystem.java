
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
  
  public ArmSubsystem() {
    arm = new TalonFX(CanIds.arm.id);
    encoder.setPositionOffset(0.144);
    arm.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,20,20,0.5));
    arm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,20,20,0.5));
  }
  public void setPower(double power) {
    arm.set(ControlMode.PercentOutput, power);
  }
  public void coast() {
    arm.setNeutralMode(NeutralMode.Coast);
  }
  public void brake() {
    arm.setNeutralMode(NeutralMode.Brake);
  }
  public double getPosition() {
    return encoder.getAbsolutePosition() - encoder.getPositionOffset();
  }
  public boolean isConnected() {
    return encoder.isConnected();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Absolute Encoder Angle", getPosition());
    SmartDashboard.putBoolean("connected", encoder.isConnected());
  }
}
