package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToPosition;
import frc.robot.subsystems.arm.RunArmWithJoystick;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.GroundIntakeSubsystem;
import frc.robot.subsystems.intake.RunGroundIntakeWithJoystick;

public class RobotContainer {
  private final XboxController driver = new XboxController(0); //driver
  private final XboxController operator = new XboxController(1); //operator

  //Subsystems
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem(gyroSubsystem);
  // private final ArmSubsystem armSubsystem = new ArmSubsystem();
  // private final GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();

  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBaseSubsystem, gyroSubsystem);
  // private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  // private final RunGroundIntakeWithJoystick runGroundIntakeWithJoystick = new RunGroundIntakeWithJoystick(groundIntakeSubsystem, driver);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // new JoystickButton(driver, XboxController.Button.kA.value)
    //   .whileTrue(new AutoDockBangBang(gyroSubsystem, driveBaseSubsystem));

    // new JoystickButton(driver, XboxController.Button.kB.value)
    //   .whileTrue(new ArmToPosition(armSubsystem, 5000));
    
  
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(swerveDriveFieldCentric);
    // armSubsystem.setDefaultCommand(runArmWithJoystick);
    // groundIntakeSubsystem.setDefaultCommand(runGroundIntakeWithJoystick);
  }
}
