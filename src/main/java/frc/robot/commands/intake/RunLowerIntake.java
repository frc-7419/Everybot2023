// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;


import frc.robot.Constants;
import frc.robot.commands.wrist.WristToPosition;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
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
    Commands.sequence(
      new WristToPosition(wristSubsystem, Constants.WristConstants.downSetpoint),
      new RunGroundIntake(groundIntakeSubsystem),
      new WaitCommand(1),
      new WristToPosition(wristSubsystem, Constants.WristConstants.upSetpoint),
      new WaitCommand(2.5),
      Commands.parallel(new RunGroundOuttake(groundIntakeSubsystem)),
      new RunGroundOuttake(groundIntakeSubsystem))
    );
  }
}
