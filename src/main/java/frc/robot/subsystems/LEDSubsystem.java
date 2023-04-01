package frc.robot.subsystems;

import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
AddressableLED m_led;
int color_number;
AddressableLEDBuffer m_ledBuffer;
String setMode = "none";
    public LEDSubsystem() {
  
      // PWM port 9
      // Must be a PWM header, not MXP or DIO
      m_led = new AddressableLED(9);
  
      // Reuse buffer
      // Default to a length of 100, start empty output
      // Length is expensive to set, so only set it once, then just update data
      m_ledBuffer = new AddressableLEDBuffer(100);
      m_led.setLength(m_ledBuffer.getLength());
  
      // Set the data
      m_led.setData(m_ledBuffer);
      m_led.start();

      color_number = 0;
  
    }

    int offset = 0;

    public void periodic(){        
        // sets yellow or purple for game piece wanted
        if(setMode.equals("cone")){
          for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 115, 115, 0);}
        }
        else if(setMode.equals("cube")){
          for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 65, 20, 90);}
        }
        else{
            if(!inError){
              if(offset%2 == 0){
                setMovingRedWhiteBlue(offset/2);
              }
            }
            else{
              setFlashingError(offset);
            }
          offset++;
        }
        m_led.setData(m_ledBuffer);

        color_number ++;
      }

    private void setMovingRedWhiteBlue(int otherIndexOffset){
      int otherIndex = otherIndexOffset;
      for(int i = 0; i < m_ledBuffer.getLength(); i++){
        if(((i+otherIndex)/10)%3 == 0){
          m_ledBuffer.setHSV(i, 0, 255, 128);
        }
        else if(((i+otherIndex)/10)%3 == 1){
          m_ledBuffer.setHSV(i, 0, 0, 128);
        }
        else{
          m_ledBuffer.setHSV(i, 120, 255, 128);
        }
      }
    }

    private void setFlashingError(int time){
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        if(time/10%2 == 1){
          m_ledBuffer.setRGB(i, 100, 0, 0);
        }
        else{
          m_ledBuffer.setRGB(i, 0, 0, 0);
        }
      }
    }

    private void starsAndStrips(int n){
      int m_redFirstPixelHue = 0;
      int m_blueFirstPixelHue = 120;

      int white_length = (m_ledBuffer.getLength() / 3) + (m_ledBuffer.getLength() / 3);
      int blue_length = m_ledBuffer.getLength();
      int red_length = m_ledBuffer.getLength() / 3;

      // blue lights
      for(var i = n; i < blue_length ; i++){
        final var hue = m_blueFirstPixelHue;
        m_ledBuffer.setHSV(i, hue, 255, 128);
      }
      // white lights
      for(var i = (n / 3)+(n / 3); i < white_length; i++){
        m_ledBuffer.setHSV(i, 0, 0, 128);
      }
      // red lights
      for(var i = n / 3; i < red_length; i++){
        final var hue = m_redFirstPixelHue;
        m_ledBuffer.setHSV(i, hue, 255, 128);
      }

      
    }
    
    public void setTheMode(String modeString){
      setMode = modeString;
    }
    boolean inError = false;
    public void setError(boolean error){
      inError = error;
    }
    }
