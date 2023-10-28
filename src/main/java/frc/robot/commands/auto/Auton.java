package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveForward;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmWithPID;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;

public class Auton extends SequentialCommandGroup {
  public Auton(DriveBaseSubsystem driveBaseSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SwerveDriveFieldCentric swerveDriveFieldCentric) {
    addCommands(
      new ArmWithPID(armSubsystem, 3.2, 0.422, 0.2).withTimeout(2),
      (new ArmWithPID(armSubsystem, 3.2, 0.422, 0.2).alongWith(new RunIntake(intakeSubsystem, 0.3))).withTimeout(1.5),
      // new ArmWithPID(armSubsystem, 1, 0, 0.2).withTimeout(1.5), // TO TEST
      new MoveForward(driveBaseSubsystem, swerveDriveFieldCentric, 0.3, 2)
    );
  }
}
