package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class MiddleAutonomousDriving extends CommandBase {
    
    int time;
    int pointTime;
    boolean stage1Passed = false;//this is true once fully on platform
    boolean stage2Passed = false;//this is true once the platform flips
    boolean stage3Passed = false;//this is true when we are level on the ground

    double lastPitch = 0;
    double lastPitchBefore=0;
    private DriveSubsystem driveSubsystem;
    int lastStageTime;
    
    public MiddleAutonomousDriving(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        time = 0;
        driveSubsystem.drive(-.2, 0, 0, false);
        lastPitch = 0;
        lastPitchBefore=0;
        lastStageTime=0;
    }  

    @Override
    public void execute() {
        double pitch = driveSubsystem.getPitch();

        if(stage1Passed){
            if(stage2Passed){
                if(Math.abs(pitch)<0.5){
                    stage3Passed =true;
                    pointTime = time;
                }
            }
            else{ 
                if(pitch < -10){
                    stage2Passed = true;
                }
            }  
        }
        else{
            if(pitch>10){
                stage1Passed = true;
            }
        }
        
        time++;
        lastPitchBefore = lastPitch;
        lastPitch = pitch;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setX();
    }

    @Override
    public boolean isFinished(){
        if(stage1Passed && stage2Passed && stage3Passed && time-pointTime>25){
            return true;
        }
        return false;
    }

}
