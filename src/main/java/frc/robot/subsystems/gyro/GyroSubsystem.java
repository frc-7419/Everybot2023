package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;


public class GyroSubsystem extends SubsystemBase {
public Constants constant = new Constants();
  public AHRS ahrs;
  
  public GyroSubsystem() {

        SmartDashboard.putString("subsystem", "init gyro sub");
        try {
	           ahrs = new AHRS(Constants.SerialPortAHRS); 
				} catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true); 
        }

        SmartDashboard.putNumber("init angle", ahrs.getAngle());
    }

    public double getGyroAngle(){
        return ahrs.getAngle();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("gyro", this.getGyroAngle());
    }
}