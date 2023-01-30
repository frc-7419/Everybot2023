
package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.GyroSubsystem;







public class Odometry
{

    public void drive(double xSpeed, double rot) {
        // Convert our fwd/rev and rotate commands to wheel speed commands
        DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot));
    
        currentTimeStamp = Timer.getFPGATimestamp();
    
        double leftDistance = getLeftVelocityInMeters() * (currentTimeStamp - previousTimeStamp);
        double rightDistance = getRightVelocityInMeters() * (currentTimeStamp - previousTimeStamp);
    
        ld += leftDistance;
        rd += rightDistance;
    
        double leftOutput = leftPIDController.calculate(leftDistance,
            speeds.leftMetersPerSecond);
        double rightOutput = righ tPIDController.calculate(rightDistance,
            speeds.rightMetersPerSecond);
    
        var leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        var rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
    
        setLeftVoltage(leftOutput + leftFeedforward);
        setRightVoltage(rightOutput + rightFeedforward);
        // Update the pose estimator with the most recent sensor readings.
        // poseEst.update(leftDistance, rightDistance);
        // poseEst.update(ld, rd);
      }

    public void resetOdometry(Pose2d pose) {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
        poseEst.resetToPose(pose, 0, 0);
      }

      @Override
    public void periodic() {
        currentTimeStamp = Timer.getFPGATimestamp();
        double leftDistance = getLeftVelocityInMeters() * (currentTimeStamp - previousTimeStamp);
        double rightDistance = getRightVelocityInMeters() * (currentTimeStamp - previousTimeStamp);
        ld += leftDistance;
        rd += rightDistance;
    
        SmartDashboard.putNumber("Odo X Pos", getCtrlsPoseEstimate().getX());
        SmartDashboard.putNumber("Odo Y Pos", getCtrlsPoseEstimate().getY());
        SmartDashboard.putNumber("Odo Theta", getCtrlsPoseEstimate().getRotation().getDegrees());
    
        SmartDashboard.putNumber("Dist to Target", getDist());
        SmartDashboard.putNumber("Angle to Target", getAngle());
        poseEst.update(ld, rd);
        previousTimeStamp = currentTimeStamp;
      }

}

// public synchronized void readPeriodicInputs() {
//     mPeriodicIO.timestamp = Timer.getFPGATimestamp();
//     double prevLeftTicks = mPeriodicIO.left_position_ticks;
//     double prevRightTicks = mPeriodicIO.right_position_ticks;
//     mPeriodicIO.left_voltage = mLeftMaster.getAppliedOutput() * mLeftMaster.getBusVoltage();
//     mPeriodicIO.right_voltage = mRightMaster.getAppliedOutput() * mRightMaster.getBusVoltage();

//     mPeriodicIO.left_position_ticks = mLeftEncoder.get();
//     mPeriodicIO.right_position_ticks = mRightEncoder.get();
//     mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);

//     double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / Constants.kDriveEncoderPPR)
//             * Math.PI;
//     mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;

//     double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / Constants.kDriveEncoderPPR)
//             * Math.PI;
//     mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;

//     mPeriodicIO.left_velocity_ticks_per_100ms = (int) (mLeftEncoder.getRate()
//             / (10 * mLeftEncoder.getDistancePerPulse()));
//     mPeriodicIO.right_velocity_ticks_per_100ms = (int) (mRightEncoder.getRate()
//             / (10 * mRightEncoder.getDistancePerPulse()));

//     if (mCSVWriter != null) {
//         mCSVWriter.add(mPeriodicIO);
//     }
// }
