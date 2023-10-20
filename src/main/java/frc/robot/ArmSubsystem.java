
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#encoders-software

public class ArmSubsystem extends SubsystemBase {
  TalonFX arm;
  DutyCycleEncoder encoder;
  public ArmSubsystem() {
    arm = new TalonFX(1000); //CAN?
    encoder = new DutyCycleEncoder(0); // CAN?
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

  public double getAngle() {
    return encoder.getPositionOffset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Absolute Encoder Angle", getAngle());
  }
}
