// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  public AddressableLED led;
  public AddressableLEDBuffer ledBuffer;
  /** Creates a new LedSubsystem. */
  public LedSubsystem() {
    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(23);
    led.setLength(ledBuffer.getLength());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public AddressableLED getLed(){
    return led;
  }

  public AddressableLEDBuffer getLedBuffer(){
    return ledBuffer;
  }

  public void setLEDColor(int hue, int saturation, int value){
    for (var i=0; i<ledBuffer.getLength(); i++){
      ledBuffer.setHSV(i, hue, saturation, value);
    }
    led.setData(ledBuffer);
  }

  public void startLed(){
    led.setData(ledBuffer);
    led.start();
  }

  public void stopLed(){
    led.setData(ledBuffer);
    led.stop();
  }
}
