// This subsystem is invovled with the managing of the limelight
// and getting data off it and sending it to Shuffleboard

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableInstance;



public class LimelightSubsystem {

    private void getLimelightData(){
        double limelightData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
        System.out.println(limelightData);
    }
    
    public LimelightSubsystem(){
        getLimelightData();
    }
    
    
}

