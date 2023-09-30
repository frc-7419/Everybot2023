// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.GroundIntakeConstants;

public class RunGroundIntakeUntilHolding extends CommandBase {
  private GroundIntakeSubsystem groundIntakeSubsystem;
  private boolean isIntaking;
  private boolean holdMode;
  private double lastTimeStamp;

  public RunGroundIntakeUntilHolding(GroundIntakeSubsystem groundIntakeSubsystem) {
    this.groundIntakeSubsystem = groundIntakeSubsystem;
    addRequirements(groundIntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isIntaking){
    groundIntakeSubsystem.coast();
    groundIntakeSubsystem.setAllPower(Constants.PowerConstants.groundIntakePower);
    double currentTimeStamp = Timer.getFPGATimestamp();
    double timePassed = currentTimeStamp - lastTimeStamp;
    boolean isStalling = groundIntakeSubsystem.getRightVelocity() < Constants.GroundIntakeConstants.stallVelocityThreshold || groundIntakeSubsystem.getLeftVelocity() < Constants.GroundIntakeConstants.stallVelocityThreshold ;
    boolean didDelay = timePassed > GroundIntakeConstants.gripperDelaySeconds;

    if (isStalling && didDelay) {
      // Hold mode won't be set to true unless we run it for 0.5 seconds to get the motor up to
      // speed
      holdMode = true;
      isIntaking = false;
    }
  }
    else {
      groundIntakeSubsystem.setAllPower(0);
      lastTimeStamp = Timer.getFPGATimestamp();
      groundIntakeSubsystem.brake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    groundIntakeSubsystem.setAllPower(0);
    groundIntakeSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
