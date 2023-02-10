// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class AutoDockBangBang extends CommandBase {
  private GyroSubsystem gyroSubsystem;
  private DriveBaseSubsystem driveBaseSubsystem;
  private BangBangController bangBangController;

  /** Creates a new AutoDockBangBang. */
  public AutoDockBangBang(GyroSubsystem gyroSubsystem, DriveBaseSubsystem driveBaseSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.gyroSubsystem = gyroSubsystem;
    this.driveBaseSubsystem = driveBaseSubsystem;

    bangBangController = new BangBangController();
    driveBaseSubsystem.coast();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //not sure if setpower or setvoltage, etc is good
    driveBaseSubsystem.setAllPower(bangBangController.calculate(gyroSubsystem.getPitch(), 0));
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
