package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


public class readLimelight extends CommandBase {

    private LimelightSubsystem limelight; //           pipeline number
    private boolean object_type;
    private IntakeSubsystem intakeSubsystem;
    public readLimelight(LimelightSubsystem limelight, boolean object_type){
        this.limelight = limelight;
        this.object_type = object_type;
    }

    public readLimelight(LimelightSubsystem limelight, IntakeSubsystem intake){
        this.limelight = limelight;
        intakeSubsystem = intake;
    }

    public void initialize(){
        String type = "";
        boolean objectType = object_type /*getObjectType(object_type)*/;
        // true means cone, false means cube
        if(intakeSubsystem != null){
            type = intakeSubsystem.getScoreMode();
        }
        else{
            if(objectType){
                type = "cone";
            }
            else{
                type = "cube";
            }
        }
        if(type.equals("cone")){
            objectType = true;
        }
        else{
            objectType = false;
        }

        if(objectType == true){
            limelight.setPipeline(1);
        }
        else{
            limelight.setPipeline(2);
        }
        if(checkPipeline() == false){
            System.out.println("The check failed!");
        }
        System.out.println(limelight.lime_x);
    }

    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {
        //uncomment to make sure things stop
        //drive.drive(0,0,0,false);
    }
    public boolean isFinished() {
        System.out.println("in read limelight finished");
        return true;
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
