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
        driveSubsystem.drive(.38, 0, 0, true);
        lastPitch = 0;
        lastPitchBefore=0;
        lastStageTime=0;
        pointTime = 0;
        stage1Passed = false;
        stage2Passed = false;
        stage3Passed = false;
    }  

    @Override
    public void execute() {
        double pitch = driveSubsystem.getPitch();
        if(time%5==0){
            System.out.println("pitch " + pitch);
        }
        if(stage1Passed){
            if(stage2Passed){
                if(Math.abs(pitch)<1){
                    stage3Passed =true;
                    if(pointTime == 0){
                        pointTime = time;
                    }
                }
            }
            else{ 
                if(pitch > 10){
                    stage2Passed = true;
                }
            }  
        }
        else{
            if(pitch<-10){
                stage1Passed = true;
            }
        }
        
        time++;
        lastPitchBefore = lastPitch;
        lastPitch = pitch;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished(){
        System.out.println("stage 1 passed " + stage1Passed + " stage 2 passed " + stage2Passed + " stage 3 passed " + stage3Passed);
        System.out.println("time " + time + " point time " + pointTime);
        if(stage1Passed && stage2Passed && stage3Passed && time-pointTime>5){
            return true;
        }
        return false;
    }

}
