// This subsystem is invovled with the managing of the limelight
// and getting data off it and sending it to Shuffleboard

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class LimelightSubsystem extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    private void getLimelightData(){
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        System.out.println("X = "+x);
        System.out.println("Y = "+y);
        System.out.println("Area = "+area);
    }

    public LimelightSubsystem(){
    
    }

    public void periodic() {
        System.out.println("hello");
        getLimelightData();
    }
    
    
}

