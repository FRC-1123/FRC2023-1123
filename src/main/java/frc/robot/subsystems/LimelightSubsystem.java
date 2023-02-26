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


    public NetworkTable limelight_table = NetworkTableInstance.getDefault().getTable("limelight-sauron");
    
    //public NetworkTable fudical_table =  NetworkTableInstance.getDefault().getTable("limelight-sauron");

    public NetworkTableEntry lime_tx = limelight_table.getEntry("tx");
    public NetworkTableEntry lime_ty = limelight_table.getEntry("ty");
    public NetworkTableEntry lime_ta = limelight_table.getEntry("ta");

    public double lime_x = lime_tx.getDouble(0.0);
    public double lime_y = lime_ty.getDouble(0.0);
    public double lime_area = lime_ta.getDouble(0.0);

    //public NetworkTableEntry fudical_tx = fudical_table.getEntry("tx");
    //public NetworkTableEntry fudical_ty = fudical_table.getEntry("ty");
    //public NetworkTableEntry fudical_ta = fudical_table.getEntry("ta");


    private void getLimelightData(){
        lime_x = lime_tx.getDouble(0.0);
        lime_y = lime_ty.getDouble(0.0);
        lime_area = lime_ta.getDouble(0.0);

        SmartDashboard.putNumber("X degrees", lime_x);
        SmartDashboard.putNumber("Y degrees", lime_y);
        SmartDashboard.putNumber("Area", lime_area);


        double offsetData = Math.tan(Math.toRadians(lime_x));
        SmartDashboard.putNumber("tangent", offsetData * 45);

    
    }

    public LimelightSubsystem(){
        
    }

    public double getTangent(){
        double x = lime_tx.getDouble(0.0);
        double offsetData = Math.tan(Math.toRadians(x));
        //TODO: this number is the distace from the limelight to the target IT WILL CHANGE!
        double distace = 30.5;
        return offsetData * distace;
    }

    public double getObjectOffset(){
        // read the distance from the laser sensor, caculate the offset, and return
        // for now return as if it was in the middle for testing purposes
        return 0.0;
    }

    public void setPipeline(int pipeline) {
		NetworkTableInstance.getDefault().getTable("limelight-sauron").getEntry("pipeline").setNumber(pipeline);
        // delay
        for(int i = 0; i<4; i++){
            int g = 4;
        }
    }
 

    @Override
    public void periodic() {
        // System.out.println("hello");
        getLimelightData();
        time++;

    }
    
    
}

