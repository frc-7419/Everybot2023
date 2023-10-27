// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.commands.auto.MoveForward;
// import frc.robot.commands.auto.Turn180;

// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.arm.ArmSubsystem;
// import frc.robot.subsystems.arm.ArmToPosition;
// import frc.robot.subsystems.armIntake.ArmIntakeSubsystem;
// import frc.robot.subsystems.armIntake.RunArmOuttake;
// import frc.robot.subsystems.drive.DriveBaseSubsystem;
// import frc.robot.subsystems.drive.SwerveDriveFieldCentric;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ScorePieceWithTurning extends SequentialCommandGroup {
//   /** Creates a new ScorePieceHigh. */

//   public ScorePieceWithTurning(ArmSubsystem armSubsystem, ArmIntakeSubsystem armIntakeSubsystem, SwerveDriveFieldCentric swerveDriveFieldCentric, DriveBaseSubsystem driveBaseSubsystem) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       new ArmToPosition(armSubsystem, 5),
//       new Turn180(swerveDriveFieldCentric, driveBaseSubsystem),
//       new RunArmOuttake(armIntakeSubsystem),
//       new Turn180(swerveDriveFieldCentric, driveBaseSubsystem),
//       new MoveForward(driveBaseSubsystem, swerveDriveFieldCentric)
//     );
//   }
// }
