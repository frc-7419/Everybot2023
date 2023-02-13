// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.talonsrx;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VacuumSubsystem extends SubsystemBase {
  private TalonSRX talonSRX;
  private Solenoid solenoid;
  /** Creates a new TalonSRX. */
  public VacuumSubsystem() {
    talonSRX = new TalonSRX(1);
    this.solenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power) {
    talonSRX.set(TalonSRXControlMode.PercentOutput, power);
  }
  public void setSolenoid(boolean enable) {
    solenoid.set(enable);
  }
}
