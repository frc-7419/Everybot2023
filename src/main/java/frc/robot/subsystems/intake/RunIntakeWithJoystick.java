// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PowerConstants;

public class RunIntakeWithJoystick extends CommandBase {
  /** Creates a new RunIntake. */
  private IntakeSubsystem intakeSubsystem;
  private XboxController joystick;
  public RunIntakeWithJoystick(IntakeSubsystem intakeSubsystem, XboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.joystick = joystick;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setPower(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Intake Power", intakeSubsystem.getPower());
    if (joystick.getLeftTriggerAxis() > 0) {
      intakeSubsystem.coast();
      intakeSubsystem.setPower(PowerConstants.IntakePower);
    }
    else if (joystick.getRightTriggerAxis() > 0) {
      intakeSubsystem.coast();
      intakeSubsystem.setPower(-PowerConstants.IntakePower);
    }
    else {
      intakeSubsystem.setPower(0);
      intakeSubsystem.brake();
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
