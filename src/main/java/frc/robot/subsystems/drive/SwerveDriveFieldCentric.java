// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveFieldCentric extends CommandBase {
  private SwerveDrive swerveDrive;
  private XboxController joystick;

 // Sets the joystick, driveBaseSubsystem and gyroSubsystem.
  public SwerveDriveFieldCentric(XboxController joystick, SwerveDrive swerveDrive) {
    this.joystick = joystick;
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }
  
  /*
  * How will Swerve Work?
  * Joysticks need to output a x/y speed and a rotation theta speed
  * Always REMEMBER this is FIELD-ORIENTED DRIVE
  */

  // Called when the command is initially scheduled.

  // Gets the swerve position when the command is initially scheduled
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.setModuleStatesFromJoystick(joystick);
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