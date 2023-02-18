// This subsystem is invovled with the managing of the limelight
// and getting data off it and sending it to Shuffleboard

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;




public class LimelightSubsystem extends SubsystemBase{
    //ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight Tab");
    //GenericEntry lightX;
    //NetworkTableEntry lightY;
    //NetworkTableEntry lightArea;
    int time = 0;


    public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-sauron");
    public NetworkTableEntry tx = table.getEntry("tx");
    public NetworkTableEntry ty = table.getEntry("ty");
    public NetworkTableEntry ta = table.getEntry("ta");
    private void getLimelightData(){
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        SmartDashboard.putNumber("X degrees", x);
        SmartDashboard.putNumber("Y degrees", y);
        SmartDashboard.putNumber("Area", area);


        double offsetData = Math.tan(Math.toRadians(x));
        SmartDashboard.putNumber("tangent", offsetData * 45);

    
    }

    public LimelightSubsystem(){
        
    }

    public double getTangent(){
        double x = tx.getDouble(0.0);
        double offsetData = Math.tan(Math.toRadians(x));
        //TODO: this number is the distace from the limelight to the target IT WILL CHANGE!
        double distace = 45;
        return offsetData * distace;
    }

    @Override
    public void periodic() {
        // System.out.println("hello");
        getLimelightData();
        time++;

    }
    
    
}

