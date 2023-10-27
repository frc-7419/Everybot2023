package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmWithPID;
import frc.robot.subsystems.arm.RunArmWithJoystick;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntakeWithJoystick;

public class RobotContainer {
  private final XboxController driver = new XboxController(0); //driver
  private final XboxController operator = new XboxController(1); //operator

  //Subsystems
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBaseSubsystem);
  private final RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(operator, intakeSubsystem);
  private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  private final ArmWithPID armWithPID = new ArmWithPID(armSubsystem);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //new JoystickButton(driver, Button.kB.value).onTrue(armWithPID);
    new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(armWithPID);
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    // driveBaseSubsystem.setDefaultCommand(swerveJoystickCommand);
    //driveBaseSubsystem.setDefaultCommand(swerveDriveFieldCentric);
    armSubsystem.setDefaultCommand(runArmWithJoystick);
    intakeSubsystem.setDefaultCommand(runIntakeWithJoystick); 
  }
}