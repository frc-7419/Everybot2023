
package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intake;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intake = new CANSparkMax(11, MotorType.kBrushless); //Neo 550 probably
    //maybe 11V voltage compensation
  }

  public void setPower(double power) {
    intake.set(power);
  }

  public void brake() {
    intake.setIdleMode(IdleMode.kBrake);
  }

  public void coast() {
    intake.setIdleMode(IdleMode.kCoast);
  }

  public double getPower() {
    return intake.get();
  }
}