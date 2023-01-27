// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EncoderSubsystem extends SubsystemBase {
  /** Creates a new EncoderSubsystem. */
  private Encoder encoder = new Encoder(0, 1);
  public EncoderSubsystem() {
    encoder.setDistancePerPulse(4./256.);

    // Configures the encoder to consider itself stopped when its rate is below 10
    encoder.setMinRate(10);
    
    // Reverses the direction of the encoder
    encoder.setReverseDirection(true);
    
    // Configures an encoder to average its period measurement over 5 samples
    // Can be between 1 and 127 samples
    encoder.setSamplesToAverage(5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
