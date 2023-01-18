// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sparkmax;
 
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunSparkMax extends CommandBase {
  /** Creates a new RunSparkMax. */
  private SparkMaxSubsystem sparkMaxSubsystem;
  public RunSparkMax(SparkMaxSubsystem sparkMaxSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sparkMaxSubsystem = sparkMaxSubsystem;
    addRequirements(sparkMaxSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sparkMaxSubsystem.setPower(0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sparkMaxSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
