package frc.robot.subsystems;

import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
AddressableLED m_led;
AddressableLEDBuffer m_ledBuffer;
String setMode = "none";
    public LEDSubsystem() {
  
      // PWM port 9
      // Must be a PWM header, not MXP or DIO
      m_led = new AddressableLED(9);
  
      // Reuse buffer
      // Default to a length of 60, start empty output
      // Length is expensive to set, so only set it once, then just update data
      m_ledBuffer = new AddressableLEDBuffer(150);
      m_led.setLength(m_ledBuffer.getLength());
  
      // Set the data
      m_led.setData(m_ledBuffer);
      m_led.start();
  
    }

    public void initialize(){

    }

    public void periodic(){
        // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        //     // Sets the specified LED to the RGB values for red
        //     m_ledBuffer.setRGB(i, 100, 100, 255);
        //  }
         
        // //  m_led.setData(m_ledBuffer);

        // // Fill the buffer with a rainbow
        // rainbow();
        // // Set the LEDs
        // m_led.setData(m_ledBuffer);
        
        //sets yellow or purple for game piece wanted
        if(setMode.equals("cone")){
          for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 128, 128, 0);}
        }
        else if(setMode.equals("cube")){
          for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 72, 22, 100);}
        }
        else{
          for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);}
        }
        m_led.setData(m_ledBuffer);
      }

    private void rainbow() {  
        // For every pixel
        int m_rainbowFirstPixelHue = 0;
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
    
      }
    
    public void setTheMode(String modeString){
      setMode = modeString;
    }
    }
