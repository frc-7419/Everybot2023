package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.drive.SwerveJoystickCommand;
import frc.robot.subsystems.drive.TestIndividualSwerve;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmWithPID;
import frc.robot.subsystems.arm.RunArmWithJoystick;

public class RobotContainer {
  private final XboxController driver = new XboxController(0); //driver
  private final XboxController operator = new XboxController(1); //operator

  //Subsystems
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
  // private final GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();

  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBaseSubsystem);
  private final SwerveJoystickCommand swerveJoystickCommand = new SwerveJoystickCommand(driveBaseSubsystem, driver);
  private final TestIndividualSwerve testIndividualSwerve = new TestIndividualSwerve(driveBaseSubsystem, driver);

  private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  private final ArmWithPID armWithPID = new ArmWithPID(armSubsystem, 0);
  // private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  // private final RunGroundIntakeWithJoystick runGroundIntakeWithJoystick = new RunGroundIntakeWithJoystick(groundIntakeSubsystem, driver);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(driver, Button.kB.value).onTrue(armWithPID);
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    // driveBaseSubsystem.setDefaultCommand(swerveJoystickCommand);
    driveBaseSubsystem.setDefaultCommand(swerveDriveFieldCentric);
    armSubsystem.setDefaultCommand(runArmWithJoystick);
    // groundIntakeSubsystem.setDefaultCommand(runGroundIntakeWithJoystick);
  }
}
