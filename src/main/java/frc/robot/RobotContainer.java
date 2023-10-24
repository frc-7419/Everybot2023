package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.commands.arm.RunArmWithJoystick;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
// import frc.robot.commands.intake.RunGroundIntake;
// import frc.robot.commands.intake.RunGroundIntakeUntilHolding;
// import frc.robot.commands.intake.RunGroundIntakeWithJoystick;
// import frc.robot.commands.intake.RunGroundOuttake;
// import frc.robot.commands.wrist.RunWristWithJoystick;
// import frc.robot.commands.wrist.WristToPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
// import frc.robot.subsystems.GroundIntakeSubsystem;
// import frc.robot.subsystems.WristSubsystem;


public class RobotContainer {
  
  private final XboxController driver = new XboxController(0); //driver
  private final XboxController operator = new XboxController(1); //operator
  // private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final DriveBaseSubsystem driveBase = new DriveBaseSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  // private final WristSubsystem wristSubsystem = new WristSubsystem();
  // private final RunWristWithJoystick runWristWithJoystick = new RunWristWithJoystick(wristSubsystem, driver);
  // private final GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();
  // private final RunGroundIntakeWithJoystick runGroundIntakeWithJoystick = new RunGroundIntakeWithJoystick(groundIntakeSubsystem, driver);
  // private  final RunGroundIntake runGroundIntake = new RunGroundIntake(groundIntakeSubsystem);
  // private  final RunGroundOuttake runGroundOuttake = new RunGroundOuttake(groundIntakeSubsystem);
  // private final WristToPosition wristToPosition = new WristToPosition(wristSubsystem, 5);
  // private final RunGroundIntakeUntilHolding  runGroundIntakeUntilHolding = new RunGroundIntakeUntilHolding(groundIntakeSubsystem);
  // private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);

  private SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // new JoystickButton(driver, XboxController.Button.kA.value).onTrue(runGroundIntakeUntilHolding);
  
  }

  private void configureAutoSelector() {
    // autonomousChooser.setDefaultOption("Find Dock", new findDock(driveBase));
    SmartDashboard.putData(autonomousChooser);
  }
  public Command getAutonomousCommand() {
    // dock command
    // return new 
    return autonomousChooser.getSelected();
  }

  public void setDefaultCommands() {
    driveBase.setDefaultCommand(new SwerveDriveFieldCentric(driver, driveBase));
    // armSubsystem.setDefaultCommand(runArmWithJoystick);
    // groundIntakeSubsystem.setDefaultCommand(runGroundIntakeWithJoystick);
  }
}
