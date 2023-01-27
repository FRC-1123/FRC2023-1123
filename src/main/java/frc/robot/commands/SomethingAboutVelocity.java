package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;

public class SomethingAboutVelocity extends CommandBase{
    ExampleSubsystem sample;
    GenericEntry velocity;
    public SomethingAboutVelocity(ExampleSubsystem sample, GenericEntry velocity){
        this.sample= sample;
        this.velocity= velocity;
        addRequirements(sample);

    }
    @Override
    public void initialize(){
        sample.setVelocity(0);
    }
    @Override   
    public void end(boolean interrupted){
        sample.setPercent(0);
    }

}
