package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToPosition;
import frc.robot.subsystems.arm.RunArmWithJoystick;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.DriveTrainPoseSubsystem;
import frc.robot.subsystems.gyro.AutoDockBangBang;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.wrist.RunWristWithJoystick;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristToPosition;

public class RobotContainer {
  private final XboxController driver = new XboxController(0); //driver
  private final XboxController operator = new XboxController(1); //operator
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final DriveTrainPoseSubsystem driveTrainPoseSubsystem = new DriveTrainPoseSubsystem(gyroSubsystem, driveBaseSubsystem);
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final RunArmWithJoystick runArmWithJoystick = new RunArmWithJoystick(operator, armSubsystem);
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final RunWristWithJoystick runWristWithJoystick = new RunWristWithJoystick(wristSubsystem, driver);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(driver, driveBaseSubsystem);
  

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(driver, XboxController.Button.kA.value)
      .whileTrue(new AutoDockBangBang(gyroSubsystem, driveBaseSubsystem));

    new JoystickButton(driver, XboxController.Button.kB.value)
      .whileTrue(new ArmToPosition(armSubsystem, 5000));
    
    //Wrist setpoints - needs testing
    new JoystickButton(driver, XboxController.Button.kX.value)
      .whileTrue(new WristToPosition(wristSubsystem, 0));
    new JoystickButton(driver, XboxController.Button.kY.value)
      .whileTrue(new WristToPosition(wristSubsystem, 5000));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    armSubsystem.setDefaultCommand(runArmWithJoystick);
    // GroundIntakeSubsystem.setDefaultCommand(runGroundIntake);
    wristSubsystem.setDefaultCommand(runWristWithJoystick);
  }
}
