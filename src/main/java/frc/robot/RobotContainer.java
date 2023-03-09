package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToPosition;
import frc.robot.subsystems.arm.RunArmWithJoystick;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.AutoDockBangBang;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.FollowTrajectory;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntakeWithJoystick;

public class RobotContainer {
  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  private final RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(intakeSubsystem, driver);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(driver, driveBaseSubsystem);

  private final SendableChooser<String> pathChooser = new SendableChooser<>();
  private final String paths[] = { "Test1", "Test2" };

  public RobotContainer() {
    configureButtonBindings();
    configurePathSelector();
  }

  public void configurePathSelector() {
    pathChooser.setDefaultOption(paths[0], paths[0]);
    for (String path : Arrays.copyOfRange(paths, 1, paths.length)) {
      pathChooser.addOption(path, path);
    }
    SmartDashboard.putData("Paths", pathChooser);
  }

  private void configureButtonBindings() {
    new JoystickButton(driver, XboxController.Button.kA.value)
        .whileTrue(new AutoDockBangBang(driveBaseSubsystem));

    new JoystickButton(driver, XboxController.Button.kB.value)
        .whileTrue(new ArmToPosition(armSubsystem, 5000));
  }

  public Command getAutonomousCommand() {
    return new FollowTrajectory(
      driveBaseSubsystem, 
      PathPlanner.loadPath(pathChooser.getSelected(), new PathConstraints(4, 2)), 
      true
    );
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    armSubsystem.setDefaultCommand(runArmWithJoystick);
    intakeSubsystem.setDefaultCommand(runIntakeWithJoystick);
  }
}
