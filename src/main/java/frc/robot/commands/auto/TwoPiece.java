package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.MoveForward;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmWithPID;
import frc.robot.subsystems.arm.RunArm;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.groundIntake.GroundIntakeSubsystem;
import frc.robot.subsystems.groundIntake.RunGroundIntake;
import frc.robot.subsystems.wrist.RunWrist;
import frc.robot.subsystems.wrist.WristSubsystem;

public class TwoPiece extends SequentialCommandGroup {
  public TwoPiece(DriveBaseSubsystem driveBaseSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SwerveDriveFieldCentric swerveDriveFieldCentric, GroundIntakeSubsystem groundIntakeSubsystem, WristSubsystem wristSubsystem) {
    addCommands(
      // new ArmWithPID(armSubsystem, 3.2, 0.422, 0.2).withTimeout(2),
      // (new ArmWithPID(armSubsystem, 3.2, 0.422, 0.2).alongWith(new RunIntake(intakeSubsystem, 0.3))).withTimeout(1.5),
      // // new ArmWithPID(armSubsystem, 1, 0, 0.2).withTimeout(1.5), // TO TEST
      // new RunArm(armSubsystem, -0.15).withTimeout(4),
      new RunArm(armSubsystem,-0.15).alongWith(new RunIntake(intakeSubsystem, 0.115)).withTimeout(5),
      new RunArm(armSubsystem,-0.05).alongWith(new RunIntake(intakeSubsystem, -0.5)).withTimeout(1),
      new WaitCommand(3),
      new MoveForward(driveBaseSubsystem, swerveDriveFieldCentric, -0.3, 5.69),
      // change the setpoint to whatever the proper setpoint is after runwrist is fixed
      new RunWrist(wristSubsystem, 0),
      // change to negative if it doesnt work
      new RunGroundIntake(groundIntakeSubsystem, 0.5),
      // change the setpoint to whatever the proper setpoint is after runwrist is fixed
      new RunWrist(wristSubsystem, 90),
      new MoveForward(driveBaseSubsystem, swerveDriveFieldCentric, 0.3, 5.69),
      // change the setpoint to whatever the proper setpoint is after runwrist is fixed
      new RunWrist(wristSubsystem, 0),
      // change to positive if it doesnt work
      new RunGroundIntake(groundIntakeSubsystem, -0.5),
      new MoveForward(driveBaseSubsystem, swerveDriveFieldCentric, -0.3, 4.5)
    );
  }
}
