// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.wpilibj.XboxController;

public class AutoDockBangBang extends CommandBase {

  private DriveBaseSubsystem driveBaseSubsystem;
  private PIDController yawAngleController;
  private PIDController pitchAngleController;
  private PIDController speedController;
  private XboxController joystick;
  /** Creates a new AutoDockBangBang. */
  public AutoDockBangBang(DriveBaseSubsystem driveBaseSubsystem, XboxController joystick) {
      this.joystick = joystick;
      this.driveBaseSubsystem = driveBaseSubsystem;
      addRequirements(driveBaseSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveBaseSubsystem.coast();
    yawAngleController =
        new PIDController(
            PIDConstants.BalanceAngleKp, PIDConstants.BalanceAngleKi, PIDConstants.BalanceAngleKd);
    pitchAngleController =
        new PIDController(
            PIDConstants.BalanceAngleKp, PIDConstants.BalanceAngleKi, PIDConstants.BalanceAngleKd);
    speedController =
        new PIDController(
            PIDConstants.BalanceSpeedKp, PIDConstants.BalanceSpeedKi, PIDConstants.BalanceSpeedKd);
    yawAngleController.setSetpoint(0);
    pitchAngleController.setSetpoint(0);
    speedController.setSetpoint(PIDConstants.BalanceSpeed);
    yawAngleController.setTolerance(PIDConstants.BalanceAngleKTolerance);
    pitchAngleController.setTolerance(PIDConstants.BalanceAngleKTolerance);
    speedController.setTolerance(PIDConstants.BalanceSpeedKTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = driveBaseSubsystem.getRoll();
    double calculatedDirection = pitchAngleController.calculate(pitch);
    double xSpeed = joystick.getLeftX();
    double ySpeed = joystick.getLeftY();
    double speed = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
    double calculatedPower = speedController.calculate(speed);

    double calculatedOutput = Math.copySign(calculatedPower, calculatedDirection);
    double power = -calculatedOutput - Math.copySign(PIDConstants.BalanceSpeedkF, calculatedDirection);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
