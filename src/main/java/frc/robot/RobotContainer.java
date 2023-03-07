package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.fasterxml.jackson.annotation.JsonTypeInfo.As;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.DriveTrainPoseSubsystem;
import frc.robot.subsystems.encoder.EncoderSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.autoncommands.Position1Charge;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.LowerArm;
import frc.robot.subsystems.arm.RaiseArm;





public class RobotContainer {
  private final XboxController joystick1 = new XboxController(0); //driver
  private final XboxController joystick2 = new XboxController(1); //operator
  private final static DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final static GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final DriveTrainPoseSubsystem driveTrainPoseSubsystem = new DriveTrainPoseSubsystem(gyroSubsystem, driveBaseSubsystem);
  private final double leftPower = -0.25;
  private final double rightPower = -0.25;
  static EncoderSubsystem Es = new EncoderSubsystem();
  static ArmSubsystem As = new ArmSubsystem();
  RaiseArm Rs = new RaiseArm(As);
  LowerArm Ls = new LowerArm(As);
  
  //private final ArmSubsystem armSubsystem = new ArmSubsystem(); //comment these out as we dont even have the parts built yet or ports
  //private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick1, driveBaseSubsystem, 0.6, 0.6);
  // private SendableChooser<Command> autonChooser = new SendableChooser<>();
  private static final Position1Charge position1Charge = new Position1Charge(driveBaseSubsystem, gyroSubsystem, Es, As);
  
  public RobotContainer() {
    configureButtonBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    // align turret
  }

  // private void smartDashboardBindings() {}

  private void configureAutoSelector() {
    // autonChooser.setDefaultOption("two ball", mttdTwoBall);
    // autonChooser.addOption("three ball", mttdThreeBall);
    // SmartDashboard.putData(autonChooser);
  }

  public Command getAutonomousCommand() {
    return position1Charge;

    
  }

  public void setDefaultCommands() {
    
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
  }
}
