
package frc.robot.subsystems.groundIntake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class GroundIntake extends SubsystemBase {
  private TalonFX wrist;
  private TalonSRX wristIntake1;
  private TalonSRX wristIntake2;

  /**
   * Makes a new ArmSubsystem
   */
  public GroundIntake() {
    wrist = new TalonFX(13);
    wristIntake1 = new TalonSRX(3);
    wristIntake2 = new TalonSRX(4);
    wristIntake2.follow(wristIntake1);
  }

  public void setPowerGroundIntake(double power) {
    wristIntake1.set(ControlMode.PercentOutput, power);
  }

  public void coastGroundIntake() {
    wristIntake1.setNeutralMode(NeutralMode.Coast);
  }
  /**
   * Sets the power to the motor which moves the arm back and forth
   * @param power in percent
   */
  public void setPowerWrist(double power) {
    wrist.set(ControlMode.PercentOutput, power);
  }
  /**
   * sets the mode of the motor to coast
   */
  public void coastWrist() {
    wrist.setNeutralMode(NeutralMode.Coast);
  }
  /**
   * sets the mode of the motor to brake
   */
  public void brakeWrist() {
    wrist.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
  }
}
