package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;
  public IntakeSubsystem() {
    armMotor = new CANSparkMax(Constants.CanIds.intake.id, MotorType.kBrushless);
    armMotor.setInverted(true);
    armMotor.enableVoltageCompensation(12);
  }

  public void setSpeed(double speed){
    armMotor.set(speed);
  }

  public void coast(){
    armMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }
  public void brake(){
    armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  @Override
  public void periodic() {}
}

