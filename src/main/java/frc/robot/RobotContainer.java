package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
// import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.motor.RunMotor;
import frc.robot.subsystems.motor.TalonSubsystem;
import frc.robot.subsystems.talonsrx.RunTalon;
import frc.robot.subsystems.talonsrx.VacuumSubsystem;

public class RobotContainer {
  private final XboxController joystick1 = new XboxController(0); //driver
  private final XboxController joystick2 = new XboxController(1); //operator
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  // private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  //private final ArmSubsystem armSubsystem = new ArmSubsystem(); //comment these out as we dont even have the parts built yet or ports
  //private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick1, driveBaseSubsystem, 0.6, 0.6);
  // private SendableChooser<Command> autonChooser = new SendableChooser

  private final TalonSubsystem talonSubsystem = new TalonSubsystem();
  private final RunMotor runMotor = new RunMotor(talonSubsystem, joystick1);

  private final VacuumSubsystem vacuumSubsystem = new VacuumSubsystem();
  private final RunTalon runTalon = new RunTalon(vacuumSubsystem, joystick1);
  
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
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    //driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    talonSubsystem.setDefaultCommand(runMotor);
    vacuumSubsystem.setDefaultCommand(runTalon);
  }
}
