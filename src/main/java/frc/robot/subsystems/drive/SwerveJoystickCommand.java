package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
public class SwerveJoystickCommand extends CommandBase {

    private final DriveBaseSubsystem driveBaseSubsystem;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private SwerveModule swerveModule;
    private XboxController joystick;
    public SwerveJoystickCommand(DriveBaseSubsystem driveBaseSubsystem, XboxController joystick) {
        this.driveBaseSubsystem = driveBaseSubsystem;
        // this.xSpdFunction = xSpdFunction;
        // this.ySpdFunction = ySpdFunction;
        // this.turningSpdFunction = turningSpdFunction;
        this.joystick = joystick;
        this.xLimiter = new SlewRateLimiter(Constants.SwerveModuleConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.SwerveModuleConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(Constants.SwerveModuleConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(driveBaseSubsystem);
    }


    @Override
    public void initialize() {
    }
    public void setModuleStates(SwerveModuleState[] moduleStates) {
      for (int i=0; i<4; ++i) {
        // SmartDashboard.putNumber("Setpoint Speed of Module" + String.valueOf(i), moduleStates[i].speedMetersPerSecond);
        // SmartDashboard.putNumber("Setpoint Angle of Module" + String.valueOf(i), moduleStates[i].angle.getDegrees()); 
        driveBaseSubsystem.getSwerveModule(i).setSwerveModuleState(moduleStates[i]);
      }
    }
    
    
    @Override
    public void execute() {
        // GET THE JOYSTICK INPUTS - i swear please dont get mad at me for not burying files, i dont like burying files because its good to see the code straight up for debugging
        double xSpeed = joystick.getLeftX();
        double ySpeed = joystick.getLeftY();
        double turningSpeed = joystick.getRightX();

        // deadband
        // xSpeed = Math.abs(xSpeed) > 0.05 ? xSpeed : 0.0;
        // ySpeed = Math.abs(ySpeed) > 0.05 ? ySpeed : 0.0;
        // turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;

        // 3. driving 
        xSpeed = xLimiter.calculate(xSpeed) * Constants.SwerveModuleConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.SwerveModuleConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)* Constants.SwerveModuleConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = driveBaseSubsystem.getSwerveDriveKinematics().toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveModule.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}