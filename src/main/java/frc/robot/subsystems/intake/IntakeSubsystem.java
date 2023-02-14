package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intake;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intake = new CANSparkMax(0, MotorType.kBrushless); //Neo 550 probably
    //maybe 11V voltage compensation
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power) {
    intake.set(power);;
  }
}
