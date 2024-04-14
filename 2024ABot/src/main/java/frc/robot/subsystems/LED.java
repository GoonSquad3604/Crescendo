// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  private static LED _LeftInstance;
  private static LED _RightInstance;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_length;

  public LED(int port, int num) {
    m_length = num;
    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(m_length);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }
  public void rainbow(int pixel) {

    // For every pixel
    int m_rainbowFirstPixelHue = pixel;


    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      // Calculate the hue - hue is easier for rainbows because the color

      // shape is a circle so only one value needs to precess

      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;

      // Set the value

      m_ledBuffer.setHSV(i, hue, 255, 128);

    }

    // Increase by to make the rainbow "move"

    m_rainbowFirstPixelHue += 3;

    // Check bounds

    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);

  }

  // public static LED getInstance(boolean left) {

  //   if (_LeftInstance == null&&left) {
  //     _LeftInstance = new LED(Constants.LEDConstants.IDLeft, Constants.LEDConstants.lengthLeft);
  //   } else if(_RightInstance == null&&!left){
  //   _RightInstance = new LED(Constants.LEDConstants.IDRight, Constants.LEDConstants.lengthRight);
  //   }
  //   if(left) {
  //     return _LeftInstance;
  //   }
  //   return _RightInstance;
  // }

  /** Sets Color.
   * @param r r value [0-255]
   * @param g g value [0-255]
   * @param b b value [0-255]
   */
  public void setColor(int r, int g, int b) {
    for(int i = 0; i < m_length; i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }

    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}