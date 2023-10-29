package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoDock;
import frc.robot.commands.LockModules;
import frc.robot.commands.auto.Auton;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmWithPID;
import frc.robot.subsystems.arm.ArmWithPIDTuning;
import frc.robot.subsystems.arm.RunArmWithJoystick;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntakeWithJoystick;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.leds.RunLED;

public class RobotContainer {
  private final XboxController driver = new XboxController(0); //driver
  private final XboxController operator = new XboxController(1); //operator

  //Subsystems
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LedSubsystem ledSubsystem = new LedSubsystem();

  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBaseSubsystem);
  private final RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(operator, intakeSubsystem);
  private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  private final ArmWithPIDTuning armWithPIDTuning = new ArmWithPIDTuning(armSubsystem);
  private final ArmWithPID doubleSub = new ArmWithPID(armSubsystem, 3.0, 0.15, 0.422);
  private final LockModules lockModules = new LockModules(driveBaseSubsystem, swerveDriveFieldCentric);
  private final RunLED runLed = new RunLED(operator, ledSubsystem);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //new JoystickButton(driver, Button.kB.value).onTrue(armWithPID);
    new JoystickButton(operator, XboxController.Button.kY.value).whileTrue(armWithPIDTuning);
    new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(doubleSub);
    new JoystickButton(driver, XboxController.Button.kStart.value).onTrue(new InstantCommand(driveBaseSubsystem::zeroYaw));
    new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(lockModules);
  }

  public Command getAutonomousCommand() {
    // return new WaitCommand(5);
    return new Auton(driveBaseSubsystem, armSubsystem, intakeSubsystem, swerveDriveFieldCentric);
    // return new AutoDock(driveBaseSubsystem, swerveDriveFieldCentric);
  }

  public void setDefaultCommands() {
    // driveBaseSubsystem.setDefaultCommand(swerveJoystickCommand);
    driveBaseSubsystem.setDefaultCommand(swerveDriveFieldCentric);
    armSubsystem.setDefaultCommand(runArmWithJoystick);
    intakeSubsystem.setDefaultCommand(runIntakeWithJoystick); 
    ledSubsystem.setDefaultCommand(runLed);
  }
}