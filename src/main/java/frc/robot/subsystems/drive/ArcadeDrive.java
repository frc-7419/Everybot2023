package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {
  private CanVenomDriveBaseSubsystem canVenomDriveBaseSubsystem;
  //private DriveBaseSubsystem driveBaseSubsystem;
  private double kStraight;
  private double kTurn;
  private XboxController joystick;
  
  // Limits *acceleration* not max speed; basically kD
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(100);
  // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(70);

  /**
   * These are parameters 
   * @param joystick
   * @param driveBaseSubsystem
   * @param kStraight
   * @param kTurn
   */
  public ArcadeDrive(XboxController joystick, CanVenomDriveBaseSubsystem canVenomDriveBaseSubsystem, double kStraight, double kTurn) {
    this.joystick = joystick;
    this.canVenomDriveBaseSubsystem = canVenomDriveBaseSubsystem;
    this.kStraight = kStraight;
    this.kTurn = kTurn;
    addRequirements(canVenomDriveBaseSubsystem);
}

  @Override
  public void initialize() {
    //driveBaseSubsystem.factoryResetAll();  //no such method anymore 
   //driveBaseSubsystem.setAllDefaultInversions();
    //driveBaseSubsystem.coast();
    canVenomDriveBaseSubsystem.coast(); 
  }

  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Command#execute()
   */
  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Command#execute()
   */
  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Command#execute()
   */
  @Override
  public void execute() {
    double xSpeed = -speedLimiter.calculate(joystick.getLeftY() * kStraight);
    // double zRotation = rotLimiter.calculate(joystick.getRightX() * kTurn);
    double zRotation = joystick.getRightX() * kTurn;
    
    //driveBaseSubsystem.coast();
    canVenomDriveBaseSubsystem.coast();
    // double leftPower = kTurn * joystick.getRightX() + kSlowStraight * joystick.getRightY();
    // double rightPower = -kTurn * joystick.getRightX()+ kSlowStraight * joystick.getRightY();

    // double leftPower = xSpeed + zRotation;
    // double rightPower = xSpeed - zRotation;

    double leftPower = xSpeed + zRotation;
    double rightPower = xSpeed - zRotation;

    /**  */
    canVenomDriveBaseSubsystem.setLeftPower(leftPower);
    canVenomDriveBaseSubsystem.setRightPower(rightPower);
    
  }
  
  

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    canVenomDriveBaseSubsystem.setAllPower(0);
  }

}
