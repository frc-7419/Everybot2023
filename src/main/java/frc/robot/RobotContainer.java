package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToPosition;
import frc.robot.subsystems.arm.RunArmWithJoystick;
// import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
// import frc.robot.subsystems.drive.DriveTrainPoseSubsystem;
import frc.robot.subsystems.drive.SwerveDriveFieldCentric;
import frc.robot.subsystems.drive.SwerveModule;
// import frc.robot.subsystems.gyro.AutoDockBangBang;
import frc.robot.subsystems.gyro.GyroSubsystem;
// import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntakeWithJoystick;

public class RobotContainer {
  private final XboxController driver = new XboxController(0); //driver
  private final XboxController operator = new XboxController(1); //operator
  private final DriveBaseSubsystem swerveDrive = new DriveBaseSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  // private final DriveTrainPoseSubsystem driveTrainPoseSubsystem = new DriveTrainPoseSubsystem(gyroSubsystem, driveBaseSubsystem);
  // private final ArmSubsystem armSubsystem = new ArmSubsystem();
  // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  // private final RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(intakeSubsystem, driver);


  // private final ArcadeDrive arcadeDrive = new ArcadeDrive(driver, driveBaseSubsystem);
  

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // new JoystickButton(driver, XboxController.Button.kA.value)
    //   .whileTrue(new AutoDockBangBang(gyroSubsystem, driveBaseSubsystem));

    // new JoystickButton(driver, XboxController.Button.kB.value)
      // .whileTrue(new ArmToPosition(armSubsystem, 5000));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    swerveDrive.setDefaultCommand(new SwerveDriveFieldCentric(driver, swerveDrive, gyroSubsystem));
    //driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    // armSubsystem.setDefaultCommand(runArmWithJoystick);
  }
}
