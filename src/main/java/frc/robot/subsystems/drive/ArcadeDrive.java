package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PowerConstants;

public class ArcadeDrive extends CommandBase {

  private DriveBaseSubsystem driveBaseSubsystem;
  private XboxController joystick;

  public ArcadeDrive(XboxController joystick, DriveBaseSubsystem driveBaseSubsystem) {
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
    addRequirements(driveBaseSubsystem);
  }

  @Override
  public void initialize() {
    driveBaseSubsystem.factoryResetAll();
    driveBaseSubsystem.setAllDefaultInversions();
    driveBaseSubsystem.coast();
  }

  @Override
  public void execute() {
    double xSpeed = joystick.getLeftY() * PowerConstants.DriveBaseStraight;
    double zRotation = joystick.getRightX() * PowerConstants.DriveBaseTurn;

    double leftPower = xSpeed + zRotation;
    double rightPower = xSpeed - zRotation;

    driveBaseSubsystem.coast();
    driveBaseSubsystem.setLeftPower(leftPower);
    driveBaseSubsystem.setRightPower(rightPower);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.setAllPower(0);
  }
}
