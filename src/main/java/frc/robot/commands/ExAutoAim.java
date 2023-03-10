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
    private Command move;
    private boolean move_finished;
    private SensorSubsystem sensor;
    private IntakeSubsystem intakeSubsystem;
    public ExAutoAim(LimelightSubsystem limelight, DriveSubsystem drive, SensorSubsystem sensor){
        this.limelight = limelight;
        this.drive = drive;
        this.sensor = sensor;
    }

    
    public void initialize(){
        move_finished = false;
        tangent = limelight.getLimelightTangentAuto(object_type);
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
            tangent = tangent + getObjectOffset();
        }
        if(Math.abs(tangent) <= 1){
            move_finished = true;
        }
        else{
            tangent = tangent * 0.0254;
            int direction = 0;
            if(tangent >= 0){
                direction = 270;
            }
            else{
                direction = 90;
            }
            move = new MoveASmallDistance(drive, Math.abs(tangent), direction, 0.1);

            move.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        
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
