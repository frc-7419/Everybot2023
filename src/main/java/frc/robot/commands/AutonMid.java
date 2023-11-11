package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSetpointPID;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.RunArm;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;

public class AutonMid extends SequentialCommandGroup {
  public AutonMid(DriveBaseSubsystem driveBaseSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SwerveDriveFieldCentric swerveDriveFieldCentric) {
    addCommands(
      // new ArmWithPID(armSubsystem, 3.2, 0.422, 0.2).withTimeout(2),
      // (new ArmWithPID(armSubsystem, 3.2, 0.422, 0.2).alongWith(new RunIntake(intakeSubsystem, 0.3))).withTimeout(1.5),
      // // new ArmWithPID(armSubsystem, 1, 0, 0.2).withTimeout(1.5), // TO TEST
      new ArmSetpointPID(armSubsystem, ArmConstants.highConeSetpoint).alongWith(new RunIntake(intakeSubsystem, 0.115)).withTimeout(2),
      // new RunArm(armSubsystem,-0.05).alongWith(new RunIntake(intakeSubsystem, -0.5)).withTimeout(1),
      new RunIntake(intakeSubsystem, -0.8).withTimeout(1),
      new ArmSetpointPID(armSubsystem, ArmConstants.armIdle),
      new WaitCommand(3),
      // new MoveForward(driveBaseSubsystem, swerveDriveFieldCentric, 0.5, 5),
      // Change modifier to -1 if the line above is uncommented
      new AutoDock(driveBaseSubsystem, swerveDriveFieldCentric ,-0.5)
    );
  }
}
