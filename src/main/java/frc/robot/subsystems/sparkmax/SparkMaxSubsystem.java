// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sparkmax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SparkMaxSubsystem extends SubsystemBase {
  /** Creates a new SparkMax. */
  private CANSparkMax sparkMax;
  public SparkMaxSubsystem() {
    sparkMax = new CANSparkMax(11, MotorType.kBrushless);

    sparkMax.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power) {
    sparkMax.set(power);
  }

  public CANSparkMax getSparkMax() {
    return sparkMax;
  }
}
