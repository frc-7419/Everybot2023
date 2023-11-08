// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunGroundIntakeWithJoystick extends CommandBase {
  /** Creates a new RunGroundIntakeWithJoystick. */
  // private GroundIntake groundIntake;
  private XboxController joystick;
  private GroundIntakeSubsystem groundIntakeSubsystem;

  public RunGroundIntakeWithJoystick(GroundIntakeSubsystem groundIntakeSubsystem, XboxController joystick) {
    // this.groundIntake = groundIntake;
    this.joystick = joystick;
    this.groundIntakeSubsystem = groundIntakeSubsystem;
    addRequirements(groundIntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    groundIntakeSubsystem.wristCoast();
    groundIntakeSubsystem.setWristPower(0);
    groundIntakeSubsystem.intakeCoast();
    groundIntakeSubsystem.setIntakeSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(joystick.getRightTriggerAxis())  > 0.5 ) {
      groundIntakeSubsystem.setIntakeSpeed(joystick.getRightTriggerAxis()/5);
    }  
    if ((joystick.getLeftTriggerAxis())  > 0.5 ) {
      groundIntakeSubsystem.setIntakeSpeed(-joystick.getLeftTriggerAxis()/5);
    }

    if(joystick.getRightY()>0.07){
      groundIntakeSubsystem.wristCoast();
      groundIntakeSubsystem.setWristPower(joystick.getRightY()/4);
    }
    else {
      groundIntakeSubsystem.wristBrake();
      groundIntakeSubsystem.setWristPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    groundIntakeSubsystem.wristBrake();
    groundIntakeSubsystem.setWristPower(0);
    groundIntakeSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return groundIntakeSubsystem.isWristOverheating();
  }
}
