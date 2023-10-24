// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armIntake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntakeWithJoystick extends CommandBase {
  /** Creates a new RunIntakeWithJoystick. */
  private ArmIntakeSubsystem armIntakeSubsystem;
  private XboxController joystick;
  public RunIntakeWithJoystick(ArmIntakeSubsystem armIntakeSubsystem, XboxController joystick) {
    this.armIntakeSubsystem = armIntakeSubsystem;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getLeftBumper()) {
      armIntakeSubsystem.coast();
      armIntakeSubsystem.setSpeed(0.5);
      
      // SmartDashboard.putNumber("Arm Power", joystick.getLeftY());
    }
    else if(joystick.getRightBumper()) {
      armIntakeSubsystem.coast();
      armIntakeSubsystem.setSpeed(-0.5);
    }
    else {
      //TODO: Replace the 0 with some tested constant that keeps the arm stationary - PowerConstants.armStationaryPower
      armIntakeSubsystem.setSpeed(0);
      armIntakeSubsystem.brake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
