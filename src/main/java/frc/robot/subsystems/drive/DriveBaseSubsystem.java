package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class DriveBaseSubsystem extends SubsystemBase {
    public TalonSRX left1;
    public TalonSRX right1;
	  public TalonSRX left2;
    public TalonSRX right2;
  
  public DriveBaseSubsystem() {
    left1 = new TalonSRX(CanIds.leftMast.id);
	  right1 = new TalonSRX(CanIds.rightMast.id);
	  left2 = new TalonSRX(CanIds.leftFollow.id);
    right2 = new TalonSRX(CanIds.rightFollow.id);

    factoryResetAll();

    right1.setInverted(true);
    right1.setSensorPhase(false);
    right2.setInverted(true);
    right2.setSensorPhase(false);

    left1.setInverted(false);
    left2.setInverted(false);

    left2.follow(left1);
    right2.follow(right1);

    //comment out lines 35-46 in case turns are off and there is no time to tune
    left1.configVoltageCompSaturation(11);
    left1.enableVoltageCompensation(true);

    left2.configVoltageCompSaturation(11);
    left2.enableVoltageCompensation(true);

    right1.configVoltageCompSaturation(11);
    right2.enableVoltageCompensation(true);

    right2.configVoltageCompSaturation(11);
    right2.enableVoltageCompensation(true);
  }

  @Override
  public void periodic() {
  }

  public enum TurnDirection {
    LEFT,
    RIGHT,
  }

  // accessors
  public TalonSRX getLeftMast(){return left1;}
  public TalonSRX getRightMast(){return right1;}
  public TalonSRX getLeftFollow(){return left2;}
  public TalonSRX getRightFollow(){return right2;}

  public void setLeftVoltage(double voltage){ //comment this method out as well
    left1.set(ControlMode.PercentOutput, voltage/11);
    left2.set(ControlMode.PercentOutput, voltage/11);
  }

  public void setRightVoltage(double voltage){ //comment this method out as well
    right1.set(ControlMode.PercentOutput, voltage/11);
    right2.set(ControlMode.PercentOutput, voltage/11);
  }

  public void setAllVoltage(double voltage){ //comment this method out as well
    setLeftVoltage(voltage);
    setRightVoltage(voltage);
  }

  public void setLeftPower(double power){
    left1.set(ControlMode.PercentOutput, power);
    left2.set(ControlMode.PercentOutput, power);
  }

  public void setRightPower(double power){
    right1.set(ControlMode.PercentOutput, power);
    right2.set(ControlMode.PercentOutput, power);
  }

  public void setAllPower(double power){
    setLeftPower(power);
    setRightPower(power);
  }

  public void stop(){setAllPower(0);}

  public void setAllMode(NeutralMode mode){
    right1.setNeutralMode(mode);
    right2.setNeutralMode(mode);
    left1.setNeutralMode(mode);
    left2.setNeutralMode(mode);
  }

  public void brake(){setAllMode(NeutralMode.Brake);}

  public void coast(){setAllMode(NeutralMode.Coast);}

  public double getLeftVelocity(){return left1.getSelectedSensorVelocity();}
  public double getRightVelocity(){return right1.getSelectedSensorVelocity();}

  public void setAllDefaultInversions() {
    right1.setInverted(true);
    right2.setInverted(true);
    left1.setInverted(false);
    left2.setInverted(false);
  }

  public void factoryResetAll() {
    right1.configFactoryDefault();
    right2.configFactoryDefault();
    left1.configFactoryDefault();
    left2.configFactoryDefault();
  }
}
