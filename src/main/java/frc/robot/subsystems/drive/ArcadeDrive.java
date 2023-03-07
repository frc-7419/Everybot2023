package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PowerConstants;

public class ArcadeDrive extends CommandBase {

  private DriveBaseSubsystem driveBaseSubsystem;
  private XboxController joystick;
  
  // Limits *acceleration* not max speed; basically kD
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(100);
  // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(70);

  /**
   * These are parameters 
   * @param joystick
   * @param driveBaseSubsystem
   */
  public ArcadeDrive(XboxController joystick, DriveBaseSubsystem driveBaseSubsystem) {
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
    addRequirements(driveBaseSubsystem);
}

  @Override
  public void initialize() {
    //driveBaseSubsystem.factoryResetAll();  //no such method anymore 
   //driveBaseSubsystem.setAllDefaultInversions();
    driveBaseSubsystem.coast(); 
  }

  @Override
  public void execute() {
    double xSpeed = -speedLimiter.calculate(joystick.getLeftY() * PowerConstants.DriveBaseStraight);
    // double zRotation = rotLimiter.calculate(joystick.getRightX() * kTurn);
    double zRotation = joystick.getRightX() * PowerConstants.DriveBaseTurn;
    
    driveBaseSubsystem.coast();
    
    // double leftPower = kTurn * joystick.getRightX() + kSlowStraight * joystick.getRightY();
    // double rightPower = -kTurn * joystick.getRightX()+ kSlowStraight * joystick.getRightY();

    // double leftPower = xSpeed + zRotation;
    // double rightPower = xSpeed - zRotation;

    double leftPower = xSpeed + zRotation;
    double rightPower = xSpeed - zRotation;

    /**  */
    driveBaseSubsystem.setLeftPower(leftPower);
    driveBaseSubsystem.setRightPower(rightPower);
    driveBaseSubsystem.putPositionOnDashboard();
    driveBaseSubsystem.putRPMOnDashBoard();
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
