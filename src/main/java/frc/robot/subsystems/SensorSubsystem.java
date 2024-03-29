package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class SensorSubsystem extends SubsystemBase {
    
    // private final I2C.Port i2cPort = I2C.Port.kOnboard;

    // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    private Rev2mDistanceSensor distOnboard; 

    private Rev2mDistanceSensor cubeDetection;
    XboxController copilotController;
    int time;
    boolean rumbleOn;

    double tangent;
    boolean object_type;
    private LimelightSubsystem limelight;
    // private Ultrasonic testingUltrasonic;
    
    public SensorSubsystem(LimelightSubsystem limelight, XboxController copilotController){
      distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
      this.limelight = limelight;
      this.copilotController = copilotController;

      cubeDetection = new Rev2mDistanceSensor(Port.kMXP);
      cubeDetection.setAutomaticMode(true);

      rumbleOn = false;
      // testingUltrasonic = new Ultrasonic(0, 1);
      // testingUltrasonic.setEnabled(true);
      // Ultrasonic.setAutomaticMode(true);
    }
    
    
    
    double lastDistanceValue = 0;
    @Override
    public void periodic() {

      object_type = true;
      tangent = limelight.getLimelightTangentAuto(object_type);
        if(object_type){
            tangent = tangent - getObjectOffset() /* - 1.5 */;
            //System.out.println(tangent);
            SmartDashboard.putNumber("tangent", tangent);
        }

        SmartDashboard.putNumber("limelight tangent", limelight.getTangentForTape());
      // System.out.println("laser sensor measurement");
      // System.out.println(getConeDistance());
      /**
       * The method GetColor() returns a normalized color value from the sensor and can be
       * useful if outputting the color to an RGB LED or similar. To
       * read the raw color, use GetRawColor().
       * 
       * The color sensor works best when within a few inches from an object in
       * well lit conditions (the built in LED is a big help here!). The farther
       * an object is the more light from the surroundings will bleed into the 
       * measurements and make it difficult to accurately determine its color.
       */
      // Color colorDetected = m_colorSensor.getColor();
      // Color8Bit detectedColor = new Color8Bit(colorDetected);
      /**
       * The sensor returns a aw IR value of the infrared light detected.
       */
      // double IR = m_colorSensor.getIR();
  
      /**
       * Open Smart Dashboard or Shuffleboard to see the color detected by the 
       * sensor.
       */
      // SmartDashboard.putNumber("Red", detectedColor.red);
      // SmartDashboard.putNumber("Green", detectedColor.green);
      // SmartDashboard.putNumber("Blue", detectedColor.blue);
      // SmartDashboard.putNumber("IR", IR);
      // SmartDashboard.putString("Color", detectedColor.toHexString());
  
      /**
       * In addition to RGB IR values, the color sensor can also return an 
       * infrared proximity value. The chip contains an IR led which will emit
       * IR pulses and measure the intensity of the return. When an object is 
       * close the value of the proximity will be large (max 2047 with default
       * settings) and will approach zero when the object is far away.
       * 
       * Proximity can be used to roughly approximate the distance of an object
       * or provide a threshold for when an object is close enough to provide
       * accurate color values.
       */
      // int proximity = m_colorSensor.getProximity();
  
      // SmartDashboard.putNumber("Proximity", proximity);
      // if(detectedColor.red < 100){
        //SmartDashboard.putString("red", );
      // }

       if(distOnboard.isRangeValid()) {
        // SmartDashboard.putNumber("Range Onboard", distOnboard.getRange());
        // SmartDashboard.putNumber("Timestamp Onboard", distOnboard.getTimestamp());
        lastDistanceValue = distOnboard.getRange();
        if(lastDistanceValue > 17){
          lastDistanceValue = 555.555;
        }
      }
      else{
        lastDistanceValue = 555.555;
      }

      SmartDashboard.putNumber("Cone Range", distOnboard.getRange());
      SmartDashboard.putBoolean("Cone range valid", distOnboard.isRangeValid());
      SmartDashboard.putBoolean("cone contained", isCone());

      SmartDashboard.putNumber("Cube range", cubeDetection.getRange());
      SmartDashboard.putBoolean("Cube range valid", cubeDetection.isRangeValid());
      SmartDashboard.putBoolean("cube contained", isCube());
      // SmartDashboard.putNumber("ultrasonic distance", testingUltrasonic.getRangeInches());

      if(isCube() && cubeDetection.isRangeValid()){
        if(!rumbleOn){
          rumbleOn = true;
          time = 150;
        }
        else{
          time--;
        }
      }
      else{
        rumbleOn = false;
        time = 0;
      }
      if(rumbleOn && time > 0){
        copilotController.setRumble(RumbleType.kBothRumble, 1);
      }
      else{
        copilotController.setRumble(RumbleType.kBothRumble, 0);
      }

      if(!distOnboard.isRangeValid()){
        copilotController.setRumble(RumbleType.kLeftRumble, 0.25);
      }

  }

    public double getConeDistance(){
      return lastDistanceValue;
    }

    double timeHadCube;
    public void reCreateSensor(){
      distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
      distOnboard.setAutomaticMode(true);
    }

    public boolean isCube(){
      if(cubeDetection.isRangeValid()){
        if(cubeDetection.getRange()<14){
          if(timeHadCube ==-1){
            timeHadCube = Timer.getFPGATimestamp();
          }
          if(Timer.getFPGATimestamp() - timeHadCube>0.15){
            return true;
          }
        }
        else{
          timeHadCube = -1;
        }
      }
      else{
        if(timeHadCube ==-1){
          timeHadCube = Timer.getFPGATimestamp();
        }
        if(Timer.getFPGATimestamp() - timeHadCube>0.15){
          return true;
        }
      }
      return false;
    }

    double timeHadCone;
    public boolean isCone(){
      if(distOnboard.isRangeValid()){
        if(distOnboard.getRange()<13.5){
          if(timeHadCone ==-1){
            timeHadCone = Timer.getFPGATimestamp();
          }
          if(Timer.getFPGATimestamp() - timeHadCone>0.15){
            return true;
          }
        }
        else{
          timeHadCone = -1;
        }
      }
      else{
        if(timeHadCone ==-1){
          timeHadCone = Timer.getFPGATimestamp();
        }
        if(Timer.getFPGATimestamp() - timeHadCone>0.15){
          return true;
        }
      }
      return false;
    }

    private double getObjectOffset(){
      double cone_distance = 0;
      // read the distance from the laser sensor, caculate the offset, and return
      // for now return as if it was in the middle for testing purposes

      cone_distance = getConeDistance();
      if(cone_distance == 555.555){
          cone_distance = 8.8;
      }
      cone_distance = cone_distance - 8.8 +1.5;
      return cone_distance;
  }

  }
