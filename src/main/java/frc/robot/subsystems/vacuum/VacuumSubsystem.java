package frc.robot.subsystems.vacuum;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class VacuumSubsystem extends SubsystemBase {
  private TalonSRX motor;

  public VacuumSubsystem() {
    motor = new TalonSRX(CanIds.vacuum.id);
  }

  @Override
  public void periodic() {}

  public void setPower(double power) {
    motor.set(ControlMode.PercentOutput, power);
  }
}
