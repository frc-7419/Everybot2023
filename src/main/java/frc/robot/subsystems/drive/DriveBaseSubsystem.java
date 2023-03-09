package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.RobotConstants;

public class DriveBaseSubsystem extends SubsystemBase {
  private CANVenom leftMast, leftFollow, rightMast, rightFollow;
  private AHRS gyro;

  private DifferentialDriveOdometry odometry;
  private Field2d field;

  public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(RobotConstants.kTrackWidth);

  public DriveBaseSubsystem() {
    rightFollow = new CANVenom(CanIds.driveRight2.id);
    rightMast = new CANVenom(CanIds.driveRight1.id);
    leftMast = new CANVenom(CanIds.driveLeft1.id);
    leftFollow = new CANVenom(CanIds.driveLeft2.id);

    leftMast.setInverted(false);
    leftFollow.setInverted(false);
    rightMast.setInverted(true);
    rightFollow.setInverted(true);

    leftFollow.follow(leftMast);
    rightFollow.follow(rightMast);

    resetEncoders();

    gyro = new AHRS(SerialPort.Port.kMXP);

    odometry = new DifferentialDriveOdometry(getRotation2d(), getLeftDistance(), getRightDistance());
  }

  public void resetEncoders() {
    leftMast.resetPosition();
    leftFollow.resetPosition();
    rightMast.resetPosition();
    rightFollow.resetPosition();
  }

  public CANVenom getLeftMast() {
    return leftMast;
  }

  public CANVenom getRightMast() {
    return rightMast;
  }

  public CANVenom getLeftFollow() {
    return leftFollow;
  }

  public CANVenom getRightFollow() {
    return rightFollow;
  }

  private void setAllBrakeCoastMode(CANVenom.BrakeCoastMode mode) {
    leftMast.setBrakeCoastMode(mode);
    leftFollow.setBrakeCoastMode(mode);
    rightMast.setBrakeCoastMode(mode);
    rightFollow.setBrakeCoastMode(mode);
  }

  public void coast() {
    setAllBrakeCoastMode(BrakeCoastMode.Coast);
  }

  public void brake() {
    setAllBrakeCoastMode(BrakeCoastMode.Brake);
  }

  public void setLeftVoltage(double voltage) {
    leftMast.setVoltage(voltage);
  }

  public void setRightVoltage(double voltage) {
    rightMast.setVoltage(voltage);
  }

  public void setAllVoltage(double voltage) {
    setLeftVoltage(voltage);
    setRightVoltage(voltage);
  }

  public void setLeftPower(double power) {
    leftMast.setCommand(ControlMode.Proportional, power);
  }

  public void setRightPower(double power) {
    rightMast.setCommand(ControlMode.Proportional, power);
  }

  public void setAllPower(double power) {
    setLeftPower(power);
    setRightPower(power);
  }

  private double getDisplacementMeters(CANVenom motor) {
    return (motor.getPosition() / Constants.GearConstants.ToughboxMiniRatio
        * Constants.RobotConstants.kWheelCircumference);
  }

  public double getLeftDistance() {
    return getDisplacementMeters(leftMast);
  }

  public double getRightDistance() {
    return getDisplacementMeters(rightMast);
  }

  public double getPositionMeters(CANVenom motor) {
    return motor.getPosition() * Constants.RobotConstants.kWheelCircumference;
  }

  public double getVelocityMeters(CANVenom motor) {
    return motor.getSpeed() * Constants.RobotConstants.kWheelCircumference;
  }

  public double getLeftVelocity() {
    return getVelocityMeters(leftMast);
  }

  public double getRightVelocity() {
    return getVelocityMeters(rightMast);
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void setField2dTrajectory(Trajectory trajectory) {
    field.getObject("traj").setTrajectory(trajectory);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
        getRotation2d(), getLeftDistance(), getRightDistance(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public void outputVolts(double leftVolts, double rightVolts) {
    setLeftVoltage(leftVolts);
    setRightVoltage(rightVolts);
  }

  @Override
  public void periodic() {
    odometry.update(
        getRotation2d(), getLeftDistance(), getRightDistance());
    field.setRobotPose(getPose());
  }
}
