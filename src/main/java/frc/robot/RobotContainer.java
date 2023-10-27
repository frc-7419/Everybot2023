package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.ArmSetpointPID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.led.RunLED;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.commands.auto.AutoDock;
import frc.robot.commands.auto.MoveForward;
import frc.robot.commands.auto.Turn180;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmWithPID;
import frc.robot.subsystems.arm.RunArmWithJoystick;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntakeWithJoystick;

public class RobotContainer {
  
  private final XboxController driver = new XboxController(0); //driver
  private final XboxController operator = new XboxController(1); //operator
  // private final ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();
  

  //Subsystems
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LedSubsystem ledSubsystem = new LedSubsystem();
  private final DriveBaseSubsystem driveBase = new DriveBaseSubsystem();
  // private final GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();
  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBase);
  private final RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(operator, intakeSubsystem);
  private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  private final ArmWithPID armWithPID = new ArmWithPID(armSubsystem);
  // private final ArmWithPID armWithPID = new ArmWithPID(armSubsystem, 0); RUN WITH CAUTION - COULD BREAK ARM
  // private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  // private final RunGroundIntakeWithJoystick runGroundIntakeWithJoystick = new RunGroundIntakeWithJoystick(groundIntakeSubsystem, driver);
  // private  final RunGroundIntake runGroundIntake = new RunGroundIntake(groundIntakeSubsystem);
  // private  final RunGroundOuttake runGroundOuttake = new RunGroundOuttake(groundIntakeSubsystem);
  // private final WristToPosition wristToPosition = new WristToPosition(wristSubsystem, 5);
  // private final RunGroundIntakeUntilHolding  runGroundIntakeUntilHolding = new RunGroundIntakeUntilHolding(groundIntakeSubsystem);
  // private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  private final RunLED runLED = new RunLED(operator, ledSubsystem);

  private SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(armWithPID); // testing setpoint, set on dashboard
    new JoystickButton(operator, XboxController.Button.kB.value).whileTrue(new ArmSetpointPID(armSubsystem, 0.469)); // Mid setpoint
    new JoystickButton(operator, XboxController.Button.kX.value).whileTrue(new ArmSetpointPID(armSubsystem, 0.637)); // High set point
  }

  private void configureAutoSelector() {
    // autonomousChooser.setDefaultOption("Score piece with turning", new ScorePieceWithTurning(armSubsystem, armIntakeSubsystem, swerveDriveFieldCentric, driveBase));
    // autonomousChooser.addOption("Score piece without turning", new ScorePieceWithoutTurning(armSubsystem, armIntakeSubsystem, swerveDriveFieldCentric, driveBase));
    autonomousChooser.addOption("Autodock", new AutoDock(driveBase, swerveDriveFieldCentric));
    SmartDashboard.putData(autonomousChooser);
  }
  public Command getAutonomousCommand() {
    
    return new MoveForward(driveBase, swerveDriveFieldCentric, 0.1, 1);
  }

  public void setDefaultCommands() {
    // driveBaseSubsystem.setDefaultCommand(swerveJoystickCommand);
    driveBase.setDefaultCommand(swerveDriveFieldCentric);
    armSubsystem.setDefaultCommand(runArmWithJoystick);
    intakeSubsystem.setDefaultCommand(runIntakeWithJoystick);
    ledSubsystem.setDefaultCommand(runLED);
  }
}