package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;

public class RobotContainer {
  private final XboxController joystick1 = new XboxController(0); //driver
  private final XboxController joystick2 = new XboxController(1); //operator
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  //private final ArmSubsystem armSubsystem = new ArmSubsystem(); //comment these out as we dont even have the parts built yet or ports
  //private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick1, driveBaseSubsystem, 0.6, 0.6);
  // private SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    // align turret
  }

  // private void smartDashboardBindings() {}

  private void configureAutoSelector() {
    // autonChooser.setDefaultOption("two ball", mttdTwoBall);
    // autonChooser.addOption("three ball", mttdThreeBall);
    // SmartDashboard.putData(autonChooser);
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
  }
}
