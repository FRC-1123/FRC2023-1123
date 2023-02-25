package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;


public class readLimelight extends CommandBase {

    private LimelightSubsystem limelight; //           pipeline number
    private int pipeline;
    public readLimelight(LimelightSubsystem limelight, int pipeline){
        this.limelight = limelight;
        this.pipeline = pipeline;
    }

    public void initialize(){
        boolean objectType = getObjectType();
        // true means cone, false means cube
        if(objectType == true){
            limelight.setPipeline(2);
        }
        else{
            limelight.setPipeline(1);
        }

    }

    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {
        //uncomment to make sure things stop
        //drive.drive(0,0,0,false);
    }
    public boolean isFinished() {
        return false;
    }

    private boolean getObjectType(){
        return true;
      }
      

}
