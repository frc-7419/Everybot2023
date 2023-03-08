package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX arm;
  public ArmSubsystem() {
    arm = new TalonFX(CanIds.leftFalcon1.id);
    arm.setInverted(true);
    arm.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    arm.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Position", getPosition());
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
  
  public double getVoltage() {
    return arm.getMotorOutputVoltage();
  }

  public double getPercentPower() {
    return arm.getMotorOutputPercent();
  }

  public double getPosition() {
    return arm.getSelectedSensorPosition();
  }
}
