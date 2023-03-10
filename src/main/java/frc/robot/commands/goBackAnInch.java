package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class goBackAnInch extends CommandBase {
    private DriveSubsystem drive;
    private Command move;
    private double distance;
    private int direction;
    private double speed;

    public goBackAnInch(DriveSubsystem drive, double distance, int direction, double speed){
        this.distance = distance;
        this.drive = drive;
        this.direction = direction;
        this.speed = speed;
    }

    public void initialize(){
        distance = distance * 0.0254;
        move = new MoveASmallDistance(drive, distance, direction, speed);

        move.schedule();
        System.out.println("go back an inch init");
    }

    public void execute(){
        System.out.println("Am I done? "+isFinished());
    }

    @Override
    public void end(boolean interrupted) {
        //uncomment to make sure things stop
        //drive.drive(0,0,0,false);
    }
    public boolean isFinished() {
        return !move.isScheduled();
    }
}
