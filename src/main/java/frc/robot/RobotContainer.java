package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.RunArmWithJoystick;
import frc.robot.subsystems.armIntake.ArmIntakeSubsystem;
import frc.robot.subsystems.armIntake.RunIntakeWithJoystick;
import frc.robot.subsystems.arm.ArmWithPID;

public class RobotContainer {
  private final XboxController driver = new XboxController(0); //driver
  private final XboxController operator = new XboxController(1); //operator

  //Subsystems
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();
  // private final GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();

  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBaseSubsystem);
  private final RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(armIntakeSubsystem, operator);
  private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  // private final ArmWithPID armWithPID = new ArmWithPID(armSubsystem, 0); RUN WITH CAUTION - COULD BREAK ARM
  // private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  // private final RunGroundIntakeWithJoystick runGroundIntakeWithJoystick = new RunGroundIntakeWithJoystick(groundIntakeSubsystem, driver);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //new JoystickButton(driver, Button.kB.value).onTrue(armWithPID);
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    // driveBaseSubsystem.setDefaultCommand(swerveJoystickCommand);
    //driveBaseSubsystem.setDefaultCommand(swerveDriveFieldCentric);
    // groundIntakeSubsystem.setDefaultCommand(runGroundIntakeWithJoystick);
    armSubsystem.setDefaultCommand(runArmWithJoystick);
    armIntakeSubsystem.setDefaultCommand(runIntakeWithJoystick); 
  }
}
