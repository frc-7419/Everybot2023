package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.drive.SwerveJoystickCommand;
import frc.robot.subsystems.drive.TestIndividualSwerve;

public class RobotContainer {
  private final XboxController driver = new XboxController(0); //driver
  private final XboxController operator = new XboxController(1); //operator

  //Subsystems
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  // private final ArmSubsystem armSubsystem = new ArmSubsystem();
  // private final GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();

  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBaseSubsystem);
  private final SwerveJoystickCommand swerveJoystickCommand = new SwerveJoystickCommand(driveBaseSubsystem, driver);
  private final TestIndividualSwerve testIndividualSwerve = new TestIndividualSwerve(driveBaseSubsystem, driver);
  // private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  // private final RunGroundIntakeWithJoystick runGroundIntakeWithJoystick = new RunGroundIntakeWithJoystick(groundIntakeSubsystem, driver);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  
  }

  public Command getAutonomousCommand() {
    int coords[][] = {{0,0}, {0, 10}, {10, 10}, {10, 0}};
    // TODO: I am not sure why i++ is dead code. Basically the point of this loop is to run each command in the path group
    for(int i = 0; i < coords.length; i++) {
      driveBaseSubsystem.createPathFinder(coords[i][0], coords[i][1]);
    }
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    // driveBaseSubsystem.setDefaultCommand(swerveJoystickCommand);
    driveBaseSubsystem.setDefaultCommand(swerveDriveFieldCentric);
    // armSubsystem.setDefaultCommand(runArmWithJoystick);
    // groundIntakeSubsystem.setDefaultCommand(runGroundIntakeWithJoystick);
  }
}
