package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.MoveASmallDistance;
import edu.wpi.first.wpilibj2.command.Command;


public class readLimelight extends CommandBase {

    private DriveSubsystem drive;
    private Command move;
    private LimelightSubsystem limelight; //           pipeline number
    private boolean object_type;
    public readLimelight(LimelightSubsystem limelight, boolean object_type, DriveSubsystem drive){
        this.limelight = limelight;
        this.object_type = object_type;
        this.drive = drive;
        addRequirements(drive);
    }

    public void initialize(){
        int direction = 180;
        move = new MoveASmallDistance(drive, 0.0254, direction);

        move.schedule();

        boolean objectType = object_type /*getObjectType(object_type)*/;
        // true means cone, false means cube
        if(objectType == true){
            limelight.setPipeline(1);
        }
        else{
            limelight.setPipeline(2);
        }
        if(checkPipeline() == false){
            System.out.println("The check failed!");
            System.out.println(limelight.lime_x);
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
        return true;
    }

    private boolean getObjectType(boolean type){
        return type;
      }
    private boolean checkPipeline(){
        boolean a = false;
        if(limelight.lime_x == 0.0){
            a = false;
        }
        else{
            a = true;
        }
        return a;
    }
      

}
