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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SensorSubsystem;


public class computeTangentMove extends CommandBase {
    private DriveSubsystem drive;
    private LimelightSubsystem limelight;
    private Command move;
    private SensorSubsystem sensor;
    public computeTangentMove(LimelightSubsystem limelight, DriveSubsystem drive){
        this.limelight = limelight;
        this.drive = drive;
        addRequirements(drive);
    }
    
    public void initialize(){

        double tangent = limelight.getTangent();
        double intake_object_position = sensor.getConeDistance();
        intake_object_position -= 6.25;
        double total_move = tangent + intake_object_position;

        move = generateSwerveCommand(drive.getPose(), new Pose2d(drive.getPose().getX(), drive.getPose().getY() + total_move, drive.getPose().getRotation()));

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

    private Command generateSwerveCommand(Pose2d startPosition, Pose2d endPosition){
        System.out.println(startPosition.getRotation().getDegrees() + " stuff " + endPosition.getRotation().getDegrees());
            // Create config for trajectory
            TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
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
