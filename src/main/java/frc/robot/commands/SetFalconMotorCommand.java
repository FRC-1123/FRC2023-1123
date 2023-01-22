package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;

public class SetFalconMotorCommand extends CommandBase{
    ExampleSubsystem sample;
    public SetFalconMotorCommand(ExampleSubsystem sample){
        this.sample= sample;

    }
    @Override
    public void initialize(){
        sample.setPosition(0);
    }
    @Override   
    public void end(boolean interrupted){
        sample.setPercent(0);
    }

}
