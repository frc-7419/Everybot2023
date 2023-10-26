package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.commands.arm.RunArmWithJoystick;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.led.RunLED;
// import frc.robot.commands.ScorePieceWithTurning;
// import frc.robot.commands.intake.RunGroundIntake;
// import frc.robot.commands.intake.RunGroundIntakeUntilHolding;
// import frc.robot.commands.intake.RunGroundIntakeWithJoystick;
// import frc.robot.commands.intake.RunGroundOuttake;
// import frc.robot.commands.wrist.RunWristWithJoystick;
// import frc.robot.commands.wrist.WristToPosition;
// import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
// import frc.robot.subsystems.GroundIntakeSubsystem;
// import frc.robot.subsystems.WristSubsystem;
// import frc.robot.commands.ScorePieceWithTurning;
// import frc.robot.commands.ScorePieceWithoutTurning;
import frc.robot.commands.auto.AutoDock;
// import frc.robot.subsystems.armIntake.ArmIntakeSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.RunArmWithJoystick;
import frc.robot.subsystems.armIntake.ArmIntakeSubsystem;
import frc.robot.subsystems.armIntake.RunIntakeWithJoystick;
import frc.robot.subsystems.arm.ArmWithPID;

public class RobotContainer {
  
  private final XboxController driver = new XboxController(0); //driver
  private final XboxController operator = new XboxController(1); //operator
  // private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final DriveBaseSubsystem driveBase = new DriveBaseSubsystem();
  // private final ArmSubsystem armSubsystem = new ArmSubsystem();
  // private final ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBase);

  //Subsystems
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();
  // private final GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();
  private final LedSubsystem ledSubsystem = new LedSubsystem();

  //Commands
  private final RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(armIntakeSubsystem, operator);
  private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
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
    //new JoystickButton(driver, Button.kB.value).onTrue(armWithPID);
  }

  private void configureAutoSelector() {
    // autonomousChooser.setDefaultOption("Score piece with turning", new ScorePieceWithTurning(armSubsystem, armIntakeSubsystem, swerveDriveFieldCentric, driveBase));
    // autonomousChooser.addOption("Score piece without turning", new ScorePieceWithoutTurning(armSubsystem, armIntakeSubsystem, swerveDriveFieldCentric, driveBase));
    autonomousChooser.addOption("Autodock", new AutoDock(driveBase, swerveDriveFieldCentric));
    SmartDashboard.putData(autonomousChooser);
  }
  public Command getAutonomousCommand() {
    
    return autonomousChooser.getSelected();
  }

  public void setDefaultCommands() {
    // driveBaseSubsystem.setDefaultCommand(swerveJoystickCommand);
    //driveBaseSubsystem.setDefaultCommand(swerveDriveFieldCentric);
    // groundIntakeSubsystem.setDefaultCommand(runGroundIntakeWithJoystick);
    armSubsystem.setDefaultCommand(runArmWithJoystick);
    armIntakeSubsystem.setDefaultCommand(runIntakeWithJoystick); 
    ledSubsystem.setDefaultCommand(runLED);
  }
}
