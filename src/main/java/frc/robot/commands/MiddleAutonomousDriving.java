package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class MiddleAutonomousDriving extends CommandBase {
    
    int time;
    double pointTime;
    boolean stage1Passed = false;//this is true once fully on platform
    boolean stage2Passed = false;//this is true once the platform flips
    boolean stage3Passed = false;//this is true when we are level on the ground
    boolean facingDriver;

    double lastPitch = 0;
    double lastPitchBefore=0;
    private DriveSubsystem driveSubsystem;
    int lastStageTime;
    PIDController m_RotationController;
    double startTime;
    
    public MiddleAutonomousDriving(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        facingDriver = true;
        addRequirements(driveSubsystem);
    }

    public MiddleAutonomousDriving(DriveSubsystem driveSubsystem, boolean facingDriver){
        this.facingDriver = facingDriver;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_RotationController = new PIDController(0.03, 0, 0);
        m_RotationController.enableContinuousInput(-180, 180);
        m_RotationController.setSetpoint(0);
        time = 0;
        driveSubsystem.drive(.4, 0, 0, true);
        lastPitch = 0;
        lastPitchBefore=0;
        lastStageTime=0;
        pointTime = 0;
        stage1Passed = false;
        stage2Passed = false;
        stage3Passed = false;
        startTime = Timer.getFPGATimestamp();
    }  

    @Override
    public void execute() {
        driveSubsystem.drive(.4, 0, m_RotationController.calculate(driveSubsystem.getPose().getRotation().getDegrees()), true);
        double pitch = 0;//driveSubsystem.getPitch();
        if(facingDriver){
            pitch = driveSubsystem.getPitch();
        }
        else{
            pitch = -driveSubsystem.getPitch();
        }
        if(time%5==0){
            System.out.println("pitch " + pitch);
        }
        if(stage1Passed){
            if(stage2Passed){
                if(Math.abs(pitch)<2.2){
                    stage3Passed =true;
                }
            }
            else{ 
                if(pitch > 7){
                    stage2Passed = true;
                }
            }  
        }
        else{
            if(pitch<-7){
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
        System.out.println("pitch " + driveSubsystem.getPitch());
        if(stage1Passed && stage2Passed && stage3Passed){
            System.out.println("in middle auto driving finished");
            return true;
        }
        if(Timer.getFPGATimestamp()-startTime > 3){
            return true;
        }
        return false;
    }

}
