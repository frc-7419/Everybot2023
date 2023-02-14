package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.RunArmWithJoystick;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.vacuum.RunVacuum;
import frc.robot.subsystems.vacuum.VacuumSubsystem;

public class RobotContainer {
  private final XboxController joystick1 = new XboxController(0); //driver
  private final XboxController joystick2 = new XboxController(1); //operator
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final VacuumSubsystem vacuumSubsystem = new VacuumSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  // private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  //private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(armSubsystem, joystick1);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick1, driveBaseSubsystem);
  
  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(joystick1, XboxController.Button.kA.value)
    .toggleOnTrue(new RunVacuum(vacuumSubsystem));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    armSubsystem.setDefaultCommand(runArmWithJoystick);
  }
}
