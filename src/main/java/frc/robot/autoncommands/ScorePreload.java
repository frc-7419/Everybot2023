// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.RaiseArm;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.MoveToSetpoint;
import frc.robot.subsystems.encoder.EncoderSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScorePreload extends ParallelCommandGroup {
  /** Creates a new Bruh. */
  public ScorePreload(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, EncoderSubsystem encoderSubsystem, ArmSubsystem armSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveToSetpoint(driveBaseSubsystem, 0.6096, encoderSubsystem),
      new RaiseArm(armSubsystem)
    );
  }
}
