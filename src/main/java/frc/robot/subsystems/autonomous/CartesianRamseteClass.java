// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/** A class where all the ramsete commands are stored */
public class CartesianRamseteClass {
    private final Drivetrain m_robotDrive;

    public CartesianRamseteClass(Drivetrain Drivetrain){
        m_robotDrive = Drivetrain;
    }

    /**
     * The Ramsete Command of the robot without any load or robot.
     * <p>
     * Has normal Characterization values.
     * @param trajectory
     */
    public RamseteCommand getCartesianRamseteCommand(Trajectory trajectory) {
        
        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory,
                m_robotDrive::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                m_robotDrive::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts,
                m_robotDrive);

        return ramseteCommand;
    }
}
