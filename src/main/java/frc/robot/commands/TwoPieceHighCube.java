package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmSetpointPID;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.RunArm;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.groundIntake.GroundIntakeSubsystem;
import frc.robot.subsystems.groundIntake.RunGroundIntake;
// import frc.robot.subsystems.wrist.RunWrist;
// import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.groundIntake.RunWrist;
import frc.robot.Constants.ArmConstants;

public class TwoPieceHighCube extends SequentialCommandGroup {
  public TwoPieceHighCube(DriveBaseSubsystem driveBaseSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SwerveDriveFieldCentric swerveDriveFieldCentric, GroundIntakeSubsystem groundIntakeSubsystem) {
    addCommands(
      // new ArmWithPID(armSubsystem, 3.2, 0.422, 0.2).withTimeout(2),
      // (new ArmWithPID(armSubsystem, 3.2, 0.422, 0.2).alongWith(new RunIntake(intakeSubsystem, 0.3))).withTimeout(1.5),
      // // new ArmWithPID(armSubsystem, 1, 0, 0.2).withTimeout(1.5), // TO TEST
      // new RunArm(armSubsystem, -0.15).withTimeout(4),
      new ArmSetpointPID(armSubsystem, ArmConstants.highCubeSetpoint).alongWith(new RunIntake(intakeSubsystem, 0.115)).withTimeout(2),
      new RunIntake(intakeSubsystem, -0.5).withTimeout(0.5),
      new ArmSetpointPID(armSubsystem,0.144),
      // change the setpoint to whatever the proper setpoint is after runwrist is fixed
      new MoveForward(driveBaseSubsystem, swerveDriveFieldCentric, -0.3, 5.69).alongWith(
        new RunWrist(groundIntakeSubsystem, 0)).alongWith(
          new RunGroundIntake(groundIntakeSubsystem, 0.5)),
      // change the setpoint to whatever the proper setpoint is after runwrist is fixed
      new RunWrist(groundIntakeSubsystem, 90).alongWith(),
      new MoveForward(driveBaseSubsystem, swerveDriveFieldCentric, -0.3, 5.69).alongWith(
        new Rotate(driveBaseSubsystem, swerveDriveFieldCentric, 180)
      ),
      // change the setpoint to whatever the proper setpoint is after runwrist is fixed
      new RunWrist(groundIntakeSubsystem, 0),
      // change to positive if it doesnt work
      new RunGroundIntake(groundIntakeSubsystem, -0.5)
    //   new MoveForward(driveBaseSubsystem, swerveDriveFieldCentric, -0.3, 4.5)
    );
  }
}
