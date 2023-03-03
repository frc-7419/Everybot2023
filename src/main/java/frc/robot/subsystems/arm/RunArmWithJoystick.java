// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunArmWithJoystick extends CommandBase {
  /** Creates a new RunArmWithJoystick. */
  private final XboxController joystick;
  private final ArmSubsystem armSubsystem;
  public RunArmWithJoystick(XboxController joystick, ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joystick = joystick;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.coast(); //not sure why coast at init but 7419 did so for elevator
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getLeftY() != 0) {
      armSubsystem.coast();
      armSubsystem.setPower(joystick.getLeftY());
    }
    else {
      armSubsystem.setPower(0);
      armSubsystem.brake();
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
