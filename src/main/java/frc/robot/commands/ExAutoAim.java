package frc.robot.commands;

import java.util.List;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.MoveASmallDistance;
import frc.robot.subsystems.SensorSubsystem;


public class ExAutoAim extends CommandBase {
    private DriveSubsystem drive;
    private LimelightSubsystem limelight;
    private double tangent;
    private boolean object_type;
    private boolean move_finished;
    private SensorSubsystem sensor;
    private IntakeSubsystem intakeSubsystem;
    public ExAutoAim(LimelightSubsystem limelight, DriveSubsystem drive, SensorSubsystem sensor, IntakeSubsystem intake){
        this.limelight = limelight;
        this.drive = drive;
        this.sensor = sensor;
        intakeSubsystem = intake;
        addRequirements(drive);
    }

    
    public void initialize(){
        move_finished = false;
        String type = intakeSubsystem.getScoreMode();
        if(type == "cone"){
            object_type = true;
        }
        else{
            object_type = false;
        }
    }

    public void execute(){
        tangent = limelight.getLimelightTangentAuto(object_type);
        if(object_type){
            tangent = tangent - getObjectOffset();
        }
        if(Math.abs(tangent) <= 1){
            move_finished = true;
        }
        else{
            // tangent = tangent * 0.0254;
            if(tangent >= 0){
                drive.drive(0, -.1, 0, false);//set speed to tangent/15
            }
            else{
                drive.drive(0, .1, 0, false);//set speed to tangent/15
            }

        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, false);
        
    }
    public boolean isFinished() {
        if(move_finished == true){
            return true;
        }
        else{
            return false;
        }
    }

    private double getObjectOffset(){
        double cone_distance = 0;
        // read the distance from the laser sensor, caculate the offset, and return
        // for now return as if it was in the middle for testing purposes

        cone_distance = sensor.getConeDistance();
        if(cone_distance == 555.555){
            cone_distance = 6.5;
        }
        cone_distance = cone_distance - 6.5;
        return cone_distance;
    }
}
