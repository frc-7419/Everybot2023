// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.encoder.EncoderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Move1Foot extends SequentialCommandGroup {
  private final double leftPower = -0.25;
  private final double rightPower = -0.25;
  EncoderSubsystem Es = new EncoderSubsystem();
  /** Creates a new Move1Foot. */
  public Move1Foot(DriveBaseSubsystem driveBaseSubsystem) {
    
    while(Es.encoder.getDistance()<1){
      driveBaseSubsystem.setLeftPower(leftPower);
    driveBaseSubsystem.setLeftPower(rightPower);
    }
    addCommands(
    );
  }
}
