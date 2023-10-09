// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestIndividualSwerve extends CommandBase {
  private DriveBaseSubsystem driveBaseSubsystem;
  private XboxController joystick;
  /** Creates a new TestIndividualSwerve. */
  public TestIndividualSwerve(DriveBaseSubsystem driveBaseSubsystem, XboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.joystick = joystick;
    addRequirements(driveBaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (int i=0; i<4; ++i) {
      driveBaseSubsystem.getSwerveModule(i).setSwerveModuleState2(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
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
