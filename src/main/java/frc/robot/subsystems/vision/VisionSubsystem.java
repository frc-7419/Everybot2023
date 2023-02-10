// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private PhotonCamera camera;
  

  public VisionSubsystem() {
    camera = new PhotonCamera("terima");
    
  }

  public Boolean hasTarget(){
    PhotonPipelineResult result = camera.getLatestResult();
    return result.hasTargets();
  }

  /**
   * Returns the 
   */
  public double getYaw() {
    PhotonPipelineResult result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    return target.getYaw();
  }
  
  @Override
  public void periodic() {
  

    // This method will be called once per scheduler run
  }
}
