package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;

public class SetFalconMotorCommand extends CommandBase{
    ExampleSubsystem sample;
    GenericEntry position;
    public SetFalconMotorCommand(ExampleSubsystem sample, GenericEntry position){
        this.sample= sample;
        this.position= position;
    }
    @Override
    public void initialize(){
        sample.setPosition(0);
    }
    
    @Override
    public void execute(){
        SmartDashboard.putNumber(, 0)
    }

    @Override   
    public void end(boolean interrupted){
        sample.setPercent(0);
    }

}
