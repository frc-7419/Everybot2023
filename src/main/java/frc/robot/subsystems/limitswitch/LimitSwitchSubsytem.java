// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimitSwitchSubsytem extends SubsystemBase {
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  public LimitSwitchSubsytem() {
    topLimitSwitch = new DigitalInput(0);
    bottomLimitSwitch = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public boolean getTopLimitSwitch(){
    return topLimitSwitch.get();
  }
  public boolean getBottomLimitSwitch(){
    return bottomLimitSwitch.get();
  }
}
