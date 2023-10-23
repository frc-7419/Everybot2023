package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;

public class RobotContainer {
  private final XboxController driver = new XboxController(0); //driver
  private final XboxController operator = new XboxController(1); //operator

  //Subsystems
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  // private final ArmSubsystem armSubsystem = new ArmSubsystem();
  // private final GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();

  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBaseSubsystem);
  // private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  // private final RunGroundIntakeWithJoystick runGroundIntakeWithJoystick = new RunGroundIntakeWithJoystick(groundIntakeSubsystem, driver);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(swerveDriveFieldCentric);
  }
}
