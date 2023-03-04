package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.MoveASmallDistance;


public class computeTangentMove extends CommandBase {
    private DriveSubsystem drive;
    private LimelightSubsystem limelight;
    private SensorSubsystem sensor;
    private boolean object_type;
    private Command move;
    public computeTangentMove(LimelightSubsystem limelight, DriveSubsystem drive, boolean object_type){
        this.limelight = limelight;
        this.object_type = object_type;
        this.drive = drive;
        addRequirements(drive);
    }
    
    public void initialize(){
        double total_move = 0;
        if(object_type == true){
            double tangent = limelight.getTangentForTape();
            double intake_object_position = getObjectOffset();
            total_move = tangent + intake_object_position;
            System.out.println("computed tangent");
            System.out.println(total_move);
        }
        else{
            double tangent = limelight.getTangentForTag();
            total_move = tangent;
        }

        // convert to meters
        total_move = total_move * 0.0254;
        int direction = 0;
        if(total_move >= 0){
            direction = 90;
        }
        else{
            direction = 270;
        }

        System.out.println(total_move);
        move = new MoveASmallDistance(drive, total_move, direction);

        move.schedule();
    }

    public void execute(){

    }

    @Override
    public void end(boolean interrupted) {
        
    }
    public boolean isFinished() {
        return !move.isScheduled();
    }

    private double getObjectOffset(){
        //double cone_distance = 0;
        // read the distance from the laser sensor, caculate the offset, and return
        // for now return as if it was in the middle for testing purposes

        //cone_distance = sensor.getConeDistance();
        

        return 0.0;
    }

    private Command generateSwerveCommand(Pose2d startPosition, Pose2d endPosition){
            // Create config for trajectory
            TrajectoryConfig config = new TrajectoryConfig(
                1,
                1)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);
        
            // An example trajectory to follow. All units in meters.
            Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startPosition,
                List.of(),
                endPosition,
                config);
        
            var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
            SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                drive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,
        
                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                drive::setModuleStates,
                drive);
            return swerveControllerCommand;
      }
}
