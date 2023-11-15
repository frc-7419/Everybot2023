package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class RobotContainer {
  
  private final XboxController driver = new XboxController(0); //driver

  //Subsystems
  private final DriveBaseSubsystem driveBase = new DriveBaseSubsystem();

  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBase);
  private SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {

  }

  private void configureAutoSelector() {
    // autonomousChooser.setDefaultOption("Score piece with turning", new ScorePieceWithTurning(armSubsystem, armIntakeSubsystem, swerveDriveFieldCentric, driveBase));
    // autonomousChooser.addOption("Score piece without turning", new ScorePieceWithoutTurning(armSubsystem, armIntakeSubsystem, swerveDriveFieldCentric, driveBase));
    // autonomousChooser.addOption("Autodock", new AutoDock(driveBase, swerveDriveFieldCentric));
    SmartDashboard.putData(autonomousChooser);
  }
  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    driveBase.setDefaultCommand(swerveDriveFieldCentric);
  }
}