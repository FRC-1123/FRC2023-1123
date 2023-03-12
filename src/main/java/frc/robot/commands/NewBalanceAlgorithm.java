package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class NewBalanceAlgorithm extends CommandBase {
    
    int time;
    int timeTickCounter;
    boolean stage1Passed = false;
    boolean stage2Passed = false;

    double lastPitch = 0;
    double lastPitchBefore=0;
    private DriveSubsystem driveSubsystem;
    int lastStageTime;
    int direction;
    
    public NewBalanceAlgorithm(DriveSubsystem driveSubsystem, int direction) {
        this.driveSubsystem = driveSubsystem;
        this.direction = direction;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        time = 0;
        driveSubsystem.drive(.15*direction, 0, 0, false);
        lastPitch = 0;
        lastPitchBefore=0;
        lastStageTime=0;
        if(Math.abs(direction)!=1){
            System.out.println("direction needs to be a 1 or -1 fix it");
            driveSubsystem.drive(0, 0, 0, false);
        }
        stage1Passed = false;
        stage2Passed = false;
    }  

    @Override
    public void execute() {

        double pitch = driveSubsystem.getPitch();
        pitch = pitch * -direction;//this is because our pitch is inverted compared to driving direction
        if(time%2==0){
            System.out.println("pitch " + pitch);
            System.out.println("delta from two pitches ago " + (pitch-lastPitchBefore));
            System.out.println("stage 1 passed " + stage1Passed + ". stage 2 passed " + stage2Passed);
        }

        if(stage1Passed){
            if(pitch >= -11 && time-lastStageTime>30){
                stage2Passed = true;
            }
        }
        else{
            if(pitch<(12*(-direction)) && pitch - lastPitchBefore >0.1){
                stage1Passed = true;
                lastStageTime = time;
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
        if(stage1Passed && stage2Passed && (driveSubsystem.getPitch()*(-direction) - lastPitchBefore > 0.2 ||
         driveSubsystem.getPitch()*(-direction)<10)){
            return true;
        }
        return false;
    }

}
