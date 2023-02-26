package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ChargeStationBalance extends CommandBase {
    
int time;
int timeTickCounter;
    private DriveSubsystem driveSubsystem;
    
    public ChargeStationBalance(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        time = 0;

    }

    @Override
    public void execute() {
        if(driveSubsystem.getPitch() > 0){
            driveSubsystem.drive(.1, 0, 0, false);
        }
        else{
            driveSubsystem.drive(-.1, 0, 0, false);
        }
        
        time++;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished(){
        double pitch = driveSubsystem.getPitch();
        if(timeTickCounter == -1 && Math.abs(pitch)< 2.5){
            timeTickCounter = time;
        }
        if(Math.abs(pitch)>= 2.5){
            timeTickCounter = -1;
        }
        
        if(Math.abs(pitch)< 2.5 && time - timeTickCounter > 10 && timeTickCounter != -1){
            return true; 
        }
        else{
            return false;
        }
        
    
    }

}
