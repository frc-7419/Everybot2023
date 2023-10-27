// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunLED extends CommandBase {
  LedSubsystem ledSubsystem;
  XboxController controller;
  /** Creates a new RunLED. */
  public RunLED(XboxController controller, LedSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledSubsystem = ledSubsystem;
    this.controller = controller;
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.startLed();
    ledSubsystem.startLed1();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller.getAButtonPressed()){
      ledSubsystem.setLEDColor(266, 89, 95);
      ledSubsystem.setLED1Color(266, 89, 95);
    }else if(controller.getBButtonPressed()){
      ledSubsystem.setLEDColor(49, 89, 95);
      ledSubsystem.setLED1Color(49, 89, 95);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSubsystem.stopLed();
    ledSubsystem.stopLed1();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
