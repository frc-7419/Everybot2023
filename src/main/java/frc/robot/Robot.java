/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.photonvision.PhotonCamera;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("photonvision");

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  XboxController xboxController = new XboxController(0);
  DriveBaseSubsystem drive = new DriveBaseSubsystem();

  // private AddressableLED led = new AddressableLED(0);
  // private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
  // private int rainbowFirstPixelHue = 0;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    
  }

  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }
  
  @Override
  public void teleopInit() {
    // robotContainer.getAutonomousCommand().cancel();
    robotContainer.setDefaultCommands();
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    double forwardSpeed;
        double rotationSpeed;

        forwardSpeed = -xboxController.getRightY();

        if (xboxController.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();

            if (result.hasTargets()) {
                // Calculate angular turn power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
            } else {
                // If we have no targets, stay still.
                rotationSpeed = 0;
            }
        } else {
            // Manual Driver Mode
            rotationSpeed = xboxController.getLeftX();
        }

       
        //needs a arcadedrive method will do later
        drive.arcadeDrive(forwardSpeed, rotationSpeed);
    
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
