// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.move1Foot;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.subsystems.encoder.EncoderSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnToSetPoint;
import frc.robot.subsystems.drive.MoveToSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Position1Charge extends SequentialCommandGroup {


  /** Creates a new ScorePreload. */
  public Position1Charge(DriveBaseSubsystem DriveBaseSubsystem, GyroSubsystem gyroSubsystem, EncoderSubsystem encoderSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // LEFTMOST POSITION FROM ALLIANCE AREA
    addCommands(
      // LEFT IS NEGATIVE RIGHT IS POSITIVE
      new TurnToSetPoint(DriveBaseSubsystem,90, gyroSubsystem),
      new MoveToSetpoint(DriveBaseSubsystem, 1, encoderSubsystem),
      new TurnToSetPoint(DriveBaseSubsystem,90, gyroSubsystem),
      // parallel
      new ScorePreload(DriveBaseSubsystem, gyroSubsystem, encoderSubsystem),
      // LOWER ARM (STILL NEEDS TO BE MADE)
      new MoveToSetpoint(DriveBaseSubsystem, -1, encoderSubsystem),
      new TurnToSetPoint(DriveBaseSubsystem, 180, gyroSubsystem),
      new MoveToSetpoint(DriveBaseSubsystem, 9, encoderSubsystem),
      new TurnToSetPoint(DriveBaseSubsystem, 90, gyroSubsystem),
      new MoveToSetpoint(DriveBaseSubsystem, 4.5, encoderSubsystem),
      new TurnToSetPoint(DriveBaseSubsystem, 90, gyroSubsystem)//,
      //new MoveToSetpoint(DriveBaseSubsystem, 2.5, encoderSubsystem)
      // The line above isn't needed if someone does auto dock.

    );
  }
}
