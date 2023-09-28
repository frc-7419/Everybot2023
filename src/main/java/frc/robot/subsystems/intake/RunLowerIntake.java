// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristToPosition;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunLowerIntake extends SequentialCommandGroup {
  /** Creates a new RunLowerIntake. */
  public RunLowerIntake(WristSubsystem wristSubsystem, GroundIntakeSubsystem groundIntakeSubsystem) {
    
    addCommands(
    Commands.parallel(
      new RunGroundIntake(groundIntakeSubsystem),
      new WaitCommand(1),
      new WristToPosition(wristSubsystem, 5)),
    new WaitCommand(2.5),
    new RunGroundOuttake(groundIntakeSubsystem));
  }
}
