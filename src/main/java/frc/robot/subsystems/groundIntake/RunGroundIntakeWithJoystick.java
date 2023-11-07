// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunGroundIntakeWithJoystick extends CommandBase {
  /** Creates a new RunGroundIntakeWithJoystick. */
  private GroundIntake groundIntake;
  private XboxController joystick;
  public RunGroundIntakeWithJoystick(GroundIntake groundIntake, XboxController joystick) {
    this.groundIntake = groundIntake;
    this.joystick = joystick;
    addRequirements(groundIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    groundIntake.coastWrist();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(joystick.getRightTriggerAxis())  > 0.01 ) {
      groundIntake.setPowerWrist(joystick.getRightTriggerAxis());
    }  
    if (-(joystick.getLeftTriggerAxis())  < -0.01 ) {
      groundIntake.setPowerWrist(-joystick.getLeftTriggerAxis());
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
