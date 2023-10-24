
package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#encoders-software
import frc.robot.Constants.CanIds;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX arm;
  private DutyCycleEncoder encoder;
  
  public ArmSubsystem() {
    arm = new TalonFX(CanIds.arm.id);
    encoder = new DutyCycleEncoder(1);
    encoder.reset();
  }

  public void setPower(double power) {
    arm.set(ControlMode.PercentOutput, power); //make a constant
  }

  public void coast() {
    arm.setNeutralMode(NeutralMode.Coast);
  }

  public void brake() {
    arm.setNeutralMode(NeutralMode.Brake);
  }

  public double getPosition() {
    return encoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Absolute Encoder Angle", getPosition());
    SmartDashboard.putBoolean("connected", encoder.isConnected());
  }
}
